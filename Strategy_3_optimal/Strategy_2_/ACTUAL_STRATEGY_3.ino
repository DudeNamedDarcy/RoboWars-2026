#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>

// for all tweaks regarding variables (small changes that could result in decent changes in the programming)
// change these if you don't like certain behavior
#define PIN_EDGE_FL      3
#define PIN_EDGE_FR      4
#define PIN_EDGE_RL      5
#define PIN_EDGE_RR      6
#define PIN_START        7    // ← confirm this — old pin 9 now conflicts with PIN_MOTOR_R
#define PIN_MOTOR_L      8
#define PIN_MOTOR_R      9

// Put your calibrated offset at the top of your main script
float tofOffsetMm = 12.0f;  // Change this to whatever your calibration function prints out, default backup is 12

// Multiplexer variable declarations
#define MUX_ADDR      0x70
#define MUX_CH_TOF_F  0
#define MUX_CH_TOF_R  1
#define MUX_CH_IMU    2

// Sensor values
#define TOF_ATTACK_MM   350
#define TOF_SCAN_MM     900
#define TOF_CLEAR_MM   1200   // RAM→SCAN exit threshold — wider than TOF_SCAN_MM to prevent oscillation
#define TOF_CONTACT_MM  120

// Motor Speeds
#define PWM_MAX         255
#define PWM_SPIN        190
#define PWM_RAM         255
#define PWM_SCAN        120
#define ACCEL_STEP_SPIN   8
#define ACCEL_STEP_RAM    6
#define ACCEL_STEP_STOP  20

// Pivot Parameters, check after
#define PIVOT_DEGREES   180.0f
#define DECEL_ZONE_DEG   45.0f
#define GYRO_SCALE     (1.0f / 131.0f)

// States
enum RobotState {
  STATE_WAIT,
  STATE_PIVOT,
  STATE_SCAN,
  STATE_RAM,
  STATE_RECOVER,
  STATE_HALT
};

// globals
RobotState state = STATE_WAIT;
Servo motorL, motorR;
Adafruit_VL53L0X tof;
MPU6050 imu(MPU6050_DEFAULT_ADDRESS, &Wire);
float    yawAccum    = 0.0f;
uint32_t tLastUs     = 0;
int16_t  gyroZOffset = 0;

// ── Start/stop interrupt ──────────────────────────────────────────────────────
// RISING  (button 2) → startRequested = true  — start or re-arm after halt
// FALLING (button 3) → haltRequested  = true  — emergency stop from any state
volatile bool haltRequested  = false;
volatile bool startRequested = false;

void ISR_startStop() {
  if (digitalRead(PIN_START) == HIGH) {
    startRequested = true;
  } else {
    haltRequested = true;
  }
}

bool edgeDetected() {
  // First read — if nothing is HIGH we can exit immediately without the debounce delay
  if (digitalRead(PIN_EDGE_FL) || digitalRead(PIN_EDGE_FR) ||
      digitalRead(PIN_EDGE_RL) || digitalRead(PIN_EDGE_RR)) {
    // Something triggered — wait 500µs and re-read to confirm it's not a noise spike
    delayMicroseconds(500);
    return (digitalRead(PIN_EDGE_FL) || digitalRead(PIN_EDGE_FR) ||
            digitalRead(PIN_EDGE_RL) || digitalRead(PIN_EDGE_RR));
  }
  return false;
}

// Here I used logic based on where the edges are being detected
// -1 = left triggered then spin right to face inward
//  1 = right triggered then spin left to face inward
//  0 = front/rear or equal then treat as centre
int edgeDirection() {
  // Short delay before reading — ensures we sample after any bounce settles
  delayMicroseconds(500);
  bool fl = digitalRead(PIN_EDGE_FL) == HIGH;
  bool fr = digitalRead(PIN_EDGE_FR) == HIGH;
  bool rl = digitalRead(PIN_EDGE_RL) == HIGH;
  bool rr = digitalRead(PIN_EDGE_RR) == HIGH;
  // Count how many left-side vs right-side sensors are triggered
  // Majority side determines which way to spin back toward center
  int leftCount  = (fl ? 1 : 0) + (rl ? 1 : 0);
  int rightCount = (fr ? 1 : 0) + (rr ? 1 : 0);
  if (leftCount > rightCount) return -1;
  if (rightCount > leftCount) return  1;
  // Diagonal tiebreaker: FL+RR or FR+RL fire together with equal counts.
  // Without this, both diagonals return 0 and the robot spins into the corner.
  if (fl && rr && !fr && !rl) return -1;
  if (fr && rl && !fl && !rr) return  1;
  return 0;
}

// I2C MUX
void muxSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);  // bitmask enables exactly one channel
  Wire.endTransmission();
  delayMicroseconds(50);  // settle time before next transmission
}

void muxDisableAll() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0x00);  // zero disables all channels
  Wire.endTransmission();
}

// Software offset calibration — averages 50 readings against a target placed
// exactly 100mm away and returns the correction to apply to all future reads.
// Call manually when needed; do NOT call from setup() during competition boot.
float calibrateVL53L0X(Adafruit_VL53L0X &sensor) {
  muxSelect(MUX_CH_TOF_F);

  Serial.println("\n--- VL53L0X Offset Calibration ---");
  Serial.println("Place target exactly 100mm from sensor. Hold still.");
  Serial.println("Starting in 5 seconds...");
  delay(5000);

  const uint32_t targetDistanceMm = 100;
  float offset = 0.0f;
  long sum = 0;
  int validSamples = 0;

  for (int i = 0; i < 50; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);
    if (measure.RangeStatus == 0) {
      sum += measure.RangeMilliMeter;
      validSamples++;
    }
    delay(30);
  }

  if (validSamples > 0) {
    float averageReading = (float)sum / validSamples;
    offset = targetDistanceMm - averageReading;
    Serial.print("Expected: 100mm | Measured avg: ");
    Serial.print(averageReading);
    Serial.print("mm | Offset: ");
    Serial.println(offset);
    Serial.println("Set tofOffsetMm to this value at the top of the file.");
  } else {
    Serial.println("FAILED: no valid readings.");
  }
  Serial.println("-----------------------------------\n");
  return offset;
}

// ToF Function used by your states
uint16_t readToF(uint8_t muxChannel) {
  muxSelect(muxChannel);
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);
  if (measure.RangeStatus == 0) {
    int correctedDist = measure.RangeMilliMeter + tofOffsetMm;
    return constrain(correctedDist, 0, 2000);
  }
  // Status 3 = min-range fail: target is closer than ~30mm sensor floor.
  // Return 0 so the calling state treats it as contact, not as clear.
  if (measure.RangeStatus == 3) return 0;
  return 2000;
}

// MOTOR Logic
void trapRamp(int& current, int target, int step) {
  // Move current toward target by at most one step per call.
  // Called every loop tick — the faster the loop, the smoother the ramp.
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

// GYRO

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
  // Store average as offset — subtracted from every future gz reading
  // so the robot doesn't think it's rotating when it's standing still
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
  // Subtract offset to get true rotation rate, scale to degrees/second,
  // then multiply by gain to get a PWM correction value
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

    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }

    if (edgeDetected()) {
      while (spinPWM > 0) {
        if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
        trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
        setMotors(spinPWM, -spinPWM);
      }
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    // Compute dt in seconds using micros() for sub-millisecond precision
    uint32_t now = micros();
    float dt = (now - tLastUs) / 1e6f;
    tLastUs  = now;

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Integrate gyro Z rate over time to track total rotation
    float rateZ = (gz - gyroZOffset) * GYRO_SCALE;
    yawAccum += rateZ * dt;

    float remaining = PIVOT_DEGREES - abs(yawAccum);
    int targetSpd;
    if (remaining < DECEL_ZONE_DEG) {
      // Inside decel zone — map remaining degrees to a lower speed
      // so treads slow down smoothly instead of slamming to a stop
      targetSpd = (int)map((long)remaining, 5, (long)DECEL_ZONE_DEG, 40, PWM_SPIN);
      targetSpd = max(targetSpd, 40);  // floor at 40 to keep moving until loop exits
    } else {
      targetSpd = PWM_SPIN;
    }

    trapRamp(spinPWM, targetSpd, ACCEL_STEP_SPIN);
    setMotors(spinPWM, -spinPWM);  // opposite signs = counter-rotate in place

    if (remaining < 90.0f) {
      // Only check ToF in second half of pivot — avoids false positives
      // while robot is still facing away from the opponent
      uint16_t dist = readToF(MUX_CH_TOF_F);
      muxSelect(MUX_CH_IMU);  // reselect IMU after ToF switches the mux
      if (dist < TOF_ATTACK_MM) {
        Serial.println("Target mid-pivot — transitioning to RAM");
        while (spinPWM > 0) {
          if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
          trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
          setMotors(spinPWM, -spinPWM);
        }
        setMotors(0, 0);
        state = STATE_RAM;
        return;
      }
    }
  }

  while (spinPWM > 0) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
    trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
    setMotors(spinPWM, -spinPWM);
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

  static int scanPWM = 0;
  uint32_t scanStart = millis();
  const uint32_t SCAN_TIMEOUT_MS = 1500;

  while (millis() - scanStart < SCAN_TIMEOUT_MS) {

    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }

    if (edgeDetected()) {
      while (scanPWM > 0) {
        if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
        trapRamp(scanPWM, 0, ACCEL_STEP_STOP);
        setMotors(scanPWM, -scanPWM);
      }
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    trapRamp(scanPWM, PWM_SCAN, ACCEL_STEP_SPIN);
    setMotors(scanPWM, -scanPWM);  // one tread forward, one back = spin in place

    uint16_t dist = readToF(MUX_CH_TOF_F);
    if (dist < TOF_SCAN_MM) {
      Serial.print("Target at ");
      Serial.print(dist);
      Serial.println(" mm — transitioning to RAM");
      while (scanPWM > 0) {
        if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
        trapRamp(scanPWM, 0, ACCEL_STEP_STOP);
        setMotors(scanPWM, -scanPWM);
      }
      setMotors(0, 0);
      state = STATE_RAM;
      return;
    }
  }

  // Timeout — exit and let loop() call executeScan() again on next tick
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

    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }

    if (edgeDetected()) {
      while (ramPWM > 0) {
        if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
        trapRamp(ramPWM, 0, ACCEL_STEP_STOP);
        setMotors(ramPWM, ramPWM);
      }
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    // Slowly ramp up — ACCEL_STEP_RAM=6 keeps torque high without tread slip
    trapRamp(ramPWM, PWM_RAM, ACCEL_STEP_RAM);

    // Apply differential correction to counteract heading drift during attack
    float correction = headingCorrection();
    int corrL = constrain((int)(ramPWM + correction), 0, PWM_MAX);
    int corrR = constrain((int)(ramPWM - correction), 0, PWM_MAX);
    setMotors(corrL, corrR);

    uint16_t dist = readToF(MUX_CH_TOF_F);

    if (dist > TOF_CLEAR_MM) {
      // Opponent escaped beyond hysteresis band — wider than TOF_SCAN_MM entry
      // threshold to prevent SCAN↔RAM oscillation near the boundary
      Serial.println("Contact lost — transitioning to SCAN");
      while (ramPWM > 0) {
        if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
        trapRamp(ramPWM, 0, ACCEL_STEP_STOP);
        setMotors(ramPWM, ramPWM);
      }
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

  // Read direction BEFORE reversing — reversing clears the sensors,
  // so if we wait we lose the information about which side triggered
  int dir = edgeDirection();

  int recPWM = 0;
  uint32_t tReverse = millis();
  while (millis() - tReverse < 350) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
    trapRamp(recPWM, 150, ACCEL_STEP_STOP);
    setMotors(-recPWM, -recPWM);  // both negative = reverse
  }

  while (recPWM > 0) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
    trapRamp(recPWM, 0, ACCEL_STEP_STOP);
    setMotors(-recPWM, -recPWM);
  }
  setMotors(0, 0);
  delay(20);

  muxSelect(MUX_CH_IMU);
  yawAccum = 0.0f;
  tLastUs  = micros();
  int pivPWM = 0;

  while (abs(yawAccum) < 90.0f) {

    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }

    if (edgeDetected()) {
      // Second boundary hit during recovery pivot — ramp down in the same
      // spin direction we were already turning, then re-enter RECOVER
      // with a fresh direction read for the new situation
      while (pivPWM > 0) {
        if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
        trapRamp(pivPWM, 0, ACCEL_STEP_STOP);
        if (dir >= 0) setMotors(-pivPWM,  pivPWM);
        else          setMotors( pivPWM, -pivPWM);
      }
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

    // Slow down in the last 20° to avoid overshooting 90°
    float rem = 90.0f - abs(yawAccum);
    int tgt = (rem < 20.0f) ? 50 : 130;
    trapRamp(pivPWM, tgt, 6);

    // dir determines which way to spin to face back toward center
    if (dir >= 0) setMotors(-pivPWM,  pivPWM);
    else          setMotors( pivPWM, -pivPWM);
  }

  while (pivPWM > 0) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
    trapRamp(pivPWM, 0, ACCEL_STEP_STOP);
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
  // Removed: while (!Serial && millis() < 3000) — causes 3s stall in competition
  // where no USB is connected. Remove for competition, restore for debugging.

  // Solenoid pins — driven LOW even though solenoid logic is removed.
  // Prevents floating gate on the MOSFET drivers (pins 44/45) from
  // coupling noise and inadvertently pulsing the MOSFET.
  pinMode(44, OUTPUT); digitalWrite(44, LOW);
  pinMode(45, OUTPUT); digitalWrite(45, LOW);

  // Edge sensor pins
  // INPUT_PULLDOWN: floating pin reads LOW, preventing false triggers if
  // a sensor is disconnected.
  pinMode(PIN_EDGE_FL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_FR, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RR, INPUT_PULLDOWN);

  pinMode(PIN_START,   INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_START), ISR_startStop, CHANGE);
  pinMode(PIN_MOTOR_L, OUTPUT);
  pinMode(PIN_MOTOR_R, OUTPUT);
  motorL.attach(PIN_MOTOR_L, 1000, 2000);
  motorR.attach(PIN_MOTOR_R, 1000, 2000);
  setMotors(0, 0);   // 1500 µs neutral — begins ESC arming sequence
  delay(2000);

  // I2C on Wire (pins 18=SDA, 19=SCL)
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);  // 3ms timeout, auto-reset bus on lockup

  muxDisableAll();

  // VL53L0X (mux ch0)
  muxSelect(MUX_CH_TOF_F);
  if (!tof.begin(VL53L0X_I2C_ADDR, false, &Wire)) {
    Serial.println("ERROR: VL53L0X not found!");
    while (1);
  }
  tof.setMeasurementTimingBudgetMicroSeconds(20000);
  tof.startRangeContinuous();
  Serial.println("VL53L0X OK");

  Serial.print("ToF offset: ");
  Serial.print(tofOffsetMm);
  Serial.println(" mm (edit tofOffsetMm at top of file to adjust)");

  // MPU-6050 (mux ch2)
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

// MAIN LOOP
void loop() {
  // Catches halt requests that arrive between blocking state calls.
  // Requests that arrive *during* a blocking call are caught by the
  // haltRequested checks inside each while loop.
  if (haltRequested) {
    setMotors(0, 0);
    state = STATE_HALT;
    haltRequested = false;
  }

  switch (state) {

    case STATE_WAIT:
      // startRequested is set by ISR on RISING edge of PIN_START (button 2)
      if (startRequested) {
        startRequested = false;
        Serial.println("Start!");
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
      // Button 2 pressed again after a halt — re-arm without needing a reset
      if (startRequested) {
        startRequested = false;
        haltRequested  = false;
        Serial.println("Re-armed — waiting for start signal");
        state = STATE_WAIT;
      }
      break;
  }
}