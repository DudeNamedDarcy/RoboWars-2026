#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

/*
  Note that a lot of these are temp values that are to be updated later since we dont currently have the robot.
  THIS IS NOT READY FOR TESTING YET
*/

// I2C addresses (to be confirmed later)
#define TCA_ADDR   0x70
#define MPU_ADDR   0x68

// Pins
#define RATCHET_PIN 4

// IR sensor pins (B = Back, F = Front)
#define IR_FL 5
#define IR_FR 6
#define IR_BL 7
#define IR_BR 8

// Start module
#define STRT_MOD 9

// Motors (left/right to be defined later)
#define MOTOR_1_PIN 10
#define MOTOR_2_PIN 11

// SDA/SCL (to be clarified later)
#define I2C_SDA 44
#define I2C_SCL 45

// ESC microsecond values (to be confirmed with motor controller)
#define ESC_FULL_FWD  2000
#define ESC_STOP      1500
#define ESC_FULL_REV  1000

// Constants (to be perfected through testing/checking with staff)
#define ENGAGE_DIST     30
#define BOUNDARY_VAL    600
#define ORBIT_1         1600
#define ORBIT_2         1800
#define REVERSAL_TIME   250
#define STARTUP_DELAY   10000

// Trapezoidal acceleration constants (to be tuned through testing)
#define ACCEL_STEP      10
#define ACCEL_DELAY     10

// Enum mode lets us switch between behaviours
enum Mode { ORBIT, CHARGE };
Mode mode = ORBIT;

// Track current ESC positions for trapezoid ramp
int currentSpd1 = ESC_STOP;
int currentSpd2 = ESC_STOP;

// Object creation of ToF distance sensor and IMU
Adafruit_VL53L0X tof;
Adafruit_MPU6050 mpu;

// Servo objects for ESC control
Servo motor1;
Servo motor2;

// MUX Communication
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire1.beginTransmission(TCA_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

// Trapezoidal Ramp
void rampTo(int target1, int target2) {
  while (currentSpd1 != target1 || currentSpd2 != target2) {
    if (currentSpd1 < target1)      currentSpd1 = min(currentSpd1 + ACCEL_STEP, target1);
    else if (currentSpd1 > target1) currentSpd1 = max(currentSpd1 - ACCEL_STEP, target1);

    if (currentSpd2 < target2)      currentSpd2 = min(currentSpd2 + ACCEL_STEP, target2);
    else if (currentSpd2 > target2) currentSpd2 = max(currentSpd2 - ACCEL_STEP, target2);

    motor1.writeMicroseconds(currentSpd1);
    motor2.writeMicroseconds(currentSpd2);
    delay(ACCEL_DELAY);
  }
}

// Motor Methods
void driveForward() { rampTo(ESC_FULL_FWD, ESC_FULL_FWD); }
void driveReverse() { rampTo(ESC_FULL_REV, ESC_FULL_REV); }
void stopMotors()   { rampTo(ESC_STOP, ESC_STOP);         }
void driveOrbit()   { rampTo(ORBIT_1, ORBIT_2);           }

void pivotLeft() {
  motor1.writeMicroseconds(ESC_FULL_REV);
  motor2.writeMicroseconds(ESC_FULL_FWD);
  currentSpd1 = ESC_FULL_REV;
  currentSpd2 = ESC_FULL_FWD;
  delay(150);
}

void pivotRight() {
  motor1.writeMicroseconds(ESC_FULL_FWD);
  motor2.writeMicroseconds(ESC_FULL_REV);
  currentSpd1 = ESC_FULL_FWD;
  currentSpd2 = ESC_FULL_REV;
  delay(150);
}

// Ratchet
void engageRatchet()  { digitalWrite(RATCHET_PIN, HIGH); }
void releaseRatchet() { digitalWrite(RATCHET_PIN, LOW);  }

// ToF Distance Reading
uint16_t readDistance() {
  tcaSelect(0);
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);
  if (measure.RangeStatus == 4) return 0;
  return measure.RangeMilliMeter / 10;
}