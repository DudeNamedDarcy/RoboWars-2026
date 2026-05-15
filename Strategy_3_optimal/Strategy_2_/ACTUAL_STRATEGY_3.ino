#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define PIN_EDGE_FL      5    // TCRT5000 digital output — front left
#define PIN_EDGE_FR      6    // TCRT5000 digital output — front right
#define PIN_EDGE_RL      7    // TCRT5000 digital output — rear left
#define PIN_EDGE_RR      8    // TCRT5000 digital output — rear right
#define PIN_START        9
#define PIN_MOTOR_L     10
#define PIN_MOTOR_R     11

// ── TCA9548A I2C multiplexer ─────────────────────────────────────────────────
#define MUX_ADDR      0x70
#define MUX_CH_TOF_F  0
#define MUX_CH_TOF_R  1
#define MUX_CH_IMU    2

// ── Sensor thresholds ────────────────────────────────────────────────────────
#define TOF_ATTACK_MM   350
#define TOF_SCAN_MM     900
#define TOF_CONTACT_MM  120

// ── Motor control ────────────────────────────────────────────────────────────
#define PWM_MAX         255
#define PWM_SPIN        190
#define PWM_RAM         255
#define PWM_SCAN        120
#define ACCEL_STEP_SPIN   8
#define ACCEL_STEP_RAM    6
#define ACCEL_STEP_STOP  20

// ── Pivot parameters ─────────────────────────────────────────────────────────
#define PIVOT_DEGREES   180.0f
#define DECEL_ZONE_DEG   45.0f
#define GYRO_SCALE     (1.0f / 131.0f)  // MPU-6050 default ±250°/s

// ── State machine ────────────────────────────────────────────────────────────
enum RobotState {
  STATE_WAIT,
  STATE_PIVOT,
  STATE_SCAN,
  STATE_RAM,
  STATE_RECOVER,
  STATE_HALT
};

// ── Globals ──────────────────────────────────────────────────────────────────
RobotState       state       = STATE_WAIT;
Servo            motorL, motorR;
Adafruit_VL53L0X tof;
MPU6050          imu(MPU6050_DEFAULT_ADDRESS, &Wire2);
float            yawAccum    = 0.0f;
uint32_t         tLastUs     = 0;
int16_t          gyroZOffset = 0;

// =============================================================================
// EDGE DETECTION — digitalRead
// WHITE LINE = DO HIGH (potentiometer-set threshold on module).
// If modules output LOW on white, flip == HIGH to == LOW in both functions.
// =============================================================================
bool edgeDetected() {
  bool fl = digitalRead(PIN_EDGE_FL);
  bool fr = digitalRead(PIN_EDGE_FR);
  bool rl = digitalRead(PIN_EDGE_RL);
  bool rr = digitalRead(PIN_EDGE_RR);

  if (fl || fr || rl || rr) {
    delayMicroseconds(500);
    return (digitalRead(PIN_EDGE_FL) || digitalRead(PIN_EDGE_FR) ||
            digitalRead(PIN_EDGE_RL) || digitalRead(PIN_EDGE_RR));
  }
  return false;
}

// -1 = left triggered  → spin right to face inward
//  1 = right triggered → spin left to face inward
//  0 = front/rear or equal → treat as centre
int edgeDirection() {
  delayMicroseconds(500);
  bool fl = digitalRead(PIN_EDGE_FL) == HIGH;
  bool fr = digitalRead(PIN_EDGE_FR) == HIGH;
  bool rl = digitalRead(PIN_EDGE_RL) == HIGH;
  bool rr = digitalRead(PIN_EDGE_RR) == HIGH;
  int leftCount  = (fl ? 1 : 0) + (rl ? 1 : 0);
  int rightCount = (fr ? 1 : 0) + (rr ? 1 : 0);
  if (leftCount > rightCount) return -1;
  if (rightCount > leftCount) return  1;
  return 0;
}

// =============================================================================
// I2C MUX
// =============================================================================
void muxSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire2.beginTransmission(MUX_ADDR);
  Wire2.write(1 << channel);
  Wire2.endTransmission();
  delayMicroseconds(50);
}

void muxDisableAll() {
  Wire2.beginTransmission(MUX_ADDR);
  Wire2.write(0x00);
  Wire2.endTransmission();
}

// =============================================================================
// ToF
// =============================================================================
uint16_t readToF(uint8_t muxChannel) {
  muxSelect(muxChannel);
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return constrain(measure.RangeMilliMeter, 0, 2000);
  }
  return 2000;
}

// =============================================================================
// MOTOR CONTROL
// =============================================================================
void trapRamp(int& current, int target, int step) {
  if (current < target) current = min(current + step, target);
  else if (current > target) current = max(current - step, target);
}

// Maps -PWM_MAX..+PWM_MAX → 1000..2000 µs (standard ESC range).
// Negative = reverse, 0 = neutral/stop, positive = forward.
void setMotors(int l, int r) {
  l = constrain(l, -PWM_MAX, PWM_MAX);
  r = constrain(r, -PWM_MAX, PWM_MAX);
  motorL.writeMicroseconds(map(l, -PWM_MAX, PWM_MAX, 1000, 2000));
  motorR.writeMicroseconds(map(r, -PWM_MAX, PWM_MAX, 1000, 2000));
}

// =============================================================================
// GYRO
// =============================================================================

// Averages 500 samples at rest to find zero-rate offset. Robot must be still.
void calibrateGyro() {
  muxSelect(MUX_CH_IMU);
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroZOffset = (int16_t)(sum / 500);
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZOffset);
}

// Positive = drifting right → reduce right motor, increase left.
// Gain 0.4f: increase toward 0.8f if robot veers, decrease toward 0.2f if it oscillates.
float headingCorrection() {
  muxSelect(MUX_CH_IMU);
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float rateZ = (gz - gyroZOffset) * GYRO_SCALE;
  return rateZ * 0.4f;
}

// =============================================================================
// STATE: PIVOT
// 180° counter-rotating pivot timed by gyro Z integration.
// Decel begins at DECEL_ZONE_DEG to prevent tread overshoot.
// Opportunistic ToF abort in second half — transitions directly to RAM if
// opponent detected at <TOF_ATTACK_MM before 180° complete.
// =============================================================================
void executePivot() {
  Serial.println("STATE: PIVOT");

  muxSelect(MUX_CH_IMU);
  yawAccum = 0.0f;
  tLastUs  = micros();
  int spinPWM = 0;

  while (abs(yawAccum) < PIVOT_DEGREES) {

    if (edgeDetected()) {
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    uint32_t now = micros();
    float dt = (now - tLastUs) / 1e6f;
    tLastUs  = now;

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rateZ = (gz - gyroZOffset) * GYRO_SCALE;
    yawAccum += rateZ * dt;

    float remaining = PIVOT_DEGREES - abs(yawAccum);
    int targetSpd;
    if (remaining < DECEL_ZONE_DEG) {
      targetSpd = (int)map((long)remaining, 5, (long)DECEL_ZONE_DEG, 40, PWM_SPIN);
      targetSpd = max(targetSpd, 40);
    } else {
      targetSpd = PWM_SPIN;
    }

    trapRamp(spinPWM, targetSpd, ACCEL_STEP_SPIN);
    setMotors(spinPWM, -spinPWM);

    if (remaining < 90.0f) {
      uint16_t dist = readToF(MUX_CH_TOF_F);
      muxSelect(MUX_CH_IMU);
      if (dist < TOF_ATTACK_MM) {
        Serial.println("Target mid-pivot — transitioning to RAM");
        setMotors(0, 0);
        state = STATE_RAM;
        return;
      }
    }
  }

  setMotors(0, 0);
  delay(30);
  Serial.print("Pivot complete. Yaw: ");
  Serial.println(yawAccum);
  state = STATE_RAM;
}

// =============================================================================
// STATE: SCAN
// Slow clockwise spin at PWM_SCAN, ToF polled each tick.
// Transitions to RAM on detection <TOF_SCAN_MM. 1500ms timeout then re-scan.
// =============================================================================
void executeScan() {
  Serial.println("STATE: SCAN");

  int scanPWM = 0;
  uint32_t scanStart = millis();
  const uint32_t SCAN_TIMEOUT_MS = 1500;

  while (millis() - scanStart < SCAN_TIMEOUT_MS) {

    if (edgeDetected()) {
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    trapRamp(scanPWM, PWM_SCAN, ACCEL_STEP_SPIN);
    setMotors(scanPWM, -scanPWM);

    uint16_t dist = readToF(MUX_CH_TOF_F);
    if (dist < TOF_SCAN_MM) {
      Serial.print("Target at ");
      Serial.print(dist);
      Serial.println(" mm — transitioning to RAM");
      setMotors(0, 0);
      state = STATE_RAM;
      return;
    }
  }

  Serial.println("Scan timeout — continuing");
}

// =============================================================================
// STATE: RAM
// Full-speed attack with trapezoidal ramp and IMU heading hold.
// Transitions to RECOVER on edge detection, SCAN on contact loss.
// =============================================================================
void executeRam() {
  Serial.println("STATE: RAM");
  int ramPWM = 0;

  while (true) {

    if (edgeDetected()) {
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    trapRamp(ramPWM, PWM_RAM, ACCEL_STEP_RAM);

    float correction = headingCorrection();
    int corrL = constrain((int)(ramPWM - correction), 0, PWM_MAX);
    int corrR = constrain((int)(ramPWM + correction), 0, PWM_MAX);
    setMotors(corrL, corrR);

    uint16_t dist = readToF(MUX_CH_TOF_F);

    if (dist > TOF_SCAN_MM) {
      Serial.println("Contact lost — transitioning to SCAN");
      setMotors(0, 0);
      state = STATE_SCAN;
      return;
    }
  }
}

// =============================================================================
// STATE: RECOVER
// Reverse 350ms away from boundary, then IMU-timed 90° pivot away from the
// triggering side. Edge detection active during pivot — re-enters RECOVER with
// a fresh direction read if a second boundary is hit.
// =============================================================================
void executeRecover() {
  Serial.println("STATE: RECOVER");

  int dir = edgeDirection();

  int recPWM = 0;
  uint32_t tReverse = millis();
  while (millis() - tReverse < 350) {
    trapRamp(recPWM, 150, ACCEL_STEP_STOP);
    setMotors(-recPWM, -recPWM);
  }

  setMotors(0, 0);
  delay(20);

  muxSelect(MUX_CH_IMU);
  yawAccum = 0.0f;
  tLastUs  = micros();
  int pivPWM = 0;

  while (abs(yawAccum) < 90.0f) {

    if (edgeDetected()) {
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    uint32_t now = micros();
    float dt = (now - tLastUs) / 1e6f;
    tLastUs  = now;

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rateZ = (gz - gyroZOffset) * GYRO_SCALE;
    yawAccum += rateZ * dt;

    float rem = 90.0f - abs(yawAccum);
    int tgt = (rem < 20.0f) ? 50 : 130;
    trapRamp(pivPWM, tgt, 6);

    if (dir >= 0) setMotors(-pivPWM,  pivPWM);
    else          setMotors( pivPWM, -pivPWM);
  }

  setMotors(0, 0);
  delay(20);
  state = STATE_SCAN;
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // ── Edge sensor pins ──────────────────────────────────────────────────────
  // INPUT_PULLDOWN: floating pin reads LOW, preventing false triggers if
  // a sensor is disconnected.
  pinMode(PIN_EDGE_FL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_FR, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RR, INPUT_PULLDOWN);

  // ── Other pins ────────────────────────────────────────────────────────────
  pinMode(PIN_START,   INPUT_PULLDOWN);
  pinMode(PIN_MOTOR_L, OUTPUT);
  pinMode(PIN_MOTOR_R, OUTPUT);
  motorL.attach(PIN_MOTOR_L, 1000, 2000);
  motorR.attach(PIN_MOTOR_R, 1000, 2000);
  setMotors(0, 0);   // 1500 µs neutral — begins ESC arming sequence
  delay(2000);

  // ── I2C on Wire2 (pins 24=SCL2, 25=SDA2) ─────────────────────────────────
  Wire2.begin();
  Wire2.setClock(400000);

  muxDisableAll();

  // ── VL53L0X (mux ch0) ────────────────────────────────────────────────────
  muxSelect(MUX_CH_TOF_F);
  if (!tof.begin(VL53L0X_I2C_ADDR, false, &Wire2)) {
    Serial.println("ERROR: VL53L0X not found!");
    while (1);
  }
  tof.setMeasurementTimingBudgetMicroSeconds(20000);
  tof.startRangeContinuous();
  Serial.println("VL53L0X OK");

  // ── MPU-6050 (mux ch2) ───────────────────────────────────────────────────
  muxSelect(MUX_CH_IMU);
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("ERROR: MPU-6050 not found!");
    while (1);
  }
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  Serial.println("MPU-6050 OK");

  Serial.println("Calibrating gyro — hold still...");
  calibrateGyro();

  Serial.println("Edge sensor check (all should be 0 on dark surface):");
  Serial.print("  FL="); Serial.print(digitalRead(PIN_EDGE_FL));
  Serial.print("  FR="); Serial.print(digitalRead(PIN_EDGE_FR));
  Serial.print("  RL="); Serial.print(digitalRead(PIN_EDGE_RL));
  Serial.print("  RR="); Serial.println(digitalRead(PIN_EDGE_RR));

  Serial.println("Ready. Waiting for start signal (pin 9)...");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  switch (state) {

    case STATE_WAIT:
      if (digitalRead(PIN_START) == HIGH) {
        Serial.println("Start!");
        delay(5000);
        state = STATE_PIVOT;
      }
      break;

    case STATE_PIVOT:
      executePivot();
      break;

    case STATE_SCAN:
      executeScan();
      break;

    case STATE_RAM:
      executeRam();
      break;

    case STATE_RECOVER:
      executeRecover();
      break;

    case STATE_HALT:
    default:
      setMotors(0, 0);
      break;
  }
}
