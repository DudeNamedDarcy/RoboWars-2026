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
#define TOF_ADDR  0x29

// Pins
#define RATCHET_PIN 4

// IR sensor pins (B = Back, F = Front), if doing analogRead, these DEFINITELY need to be changed.
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

// MUX Communication, controls 8 different i2c buses: channels 0-7
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

// Motor Methods, defining the directions of both L and R motors.
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
  
  // Print the status so we can see what's actually happening
  if (measure.RangeStatus != 0) {
    Serial.print("Status Error: "); Serial.println(measure.RangeStatus);
    return 0;
  }
  return measure.RangeMilliMeter / 10;
}



// Edge Sensor Reading
uint8_t readEdges() {
  uint8_t mask = 0;
  if (analogRead(IR_FL) > BOUNDARY_VAL) mask |= (1 << 0);
  if (analogRead(IR_FR) > BOUNDARY_VAL) mask |= (1 << 1);
  if (analogRead(IR_BL) > BOUNDARY_VAL) mask |= (1 << 2);
  if (analogRead(IR_BR) > BOUNDARY_VAL) mask |= (1 << 3);
  return mask;
}

// Setup
void setup() {
  Serial.begin(115200);

  Wire1.begin();
  Wire1.setClock(400000);

  pinMode(RATCHET_PIN, OUTPUT); releaseRatchet();
  pinMode(STRT_MOD,    INPUT_PULLUP);

  motor1.attach(MOTOR_1_PIN);
  motor2.attach(MOTOR_2_PIN);
  Serial.println("Arming ESCs...");
  motor1.writeMicroseconds(ESC_STOP);
  motor2.writeMicroseconds(ESC_STOP);
  delay(3000);
  Serial.println("ESCs armed");

  tcaSelect(0);
  if (!tof.begin(TOF_ADDR, false, &Wire1)) {
    Serial.println("ERROR: VL53L0X not found");
    while (1);
  }
  tof.startRangeContinuous();

  if (!mpu.begin(MPU_ADDR, &Wire1)) {
    Serial.println("WARNING: MPU-6050 not found");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  Serial.println("Waiting for start signal...");
  unsigned long t = millis();
  while (digitalRead(STRT_MOD) == HIGH && millis() - t < STARTUP_DELAY);

  mode = ORBIT;
  Serial.println("GO — ORBIT");
}

// Loop
void loop() {
  uint8_t edges = readEdges();
  if (edges) {
    driveReverse();
    delay(REVERSAL_TIME);
    stopMotors();

    bool leftTriggered  = (edges & 0b0101); //0b0101 = either LEFT sensor triggered
    bool rightTriggered = (edges & 0b1010); //0b1010 = either RIGHT sensor triggered

    if      (leftTriggered && !rightTriggered) pivotRight();
    else if (rightTriggered && !leftTriggered) pivotLeft();
    else                                        pivotRight();

    mode = ORBIT;
    return;
  }

  uint16_t dist = readDistance();

  if (mode == ORBIT) {
    driveOrbit();
    if (dist > 0 && dist <= ENGAGE_DIST) {
      Serial.print("Target "); Serial.print(dist); Serial.println("cm — CHARGE");
      engageRatchet();
      mode = CHARGE;
    }
  }

  if (mode == CHARGE) {
    driveForward();
    if (dist == 0 || dist > ENGAGE_DIST) {
      Serial.println("Lost — ORBIT");
      releaseRatchet();
      mode = ORBIT;
    }
  }
}