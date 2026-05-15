#include <Wire.h>
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
#define PIN_DIR_L        2    // Motor 1 direction (left tread)  LOW=fwd HIGH=rev
#define PIN_DIR_R        3    // Motor 2 direction (right tread) LOW=fwd HIGH=rev
#define PIN_SOL_A       44   // Solenoid A — push-type, engages ratchet
#define PIN_SOL_B       45   // Solenoid B — push-type, reinforces engagement

// ── Solenoid timing (ms) ─────────────────────────────────────────────────────
#define SOLENOID_ENGAGE_MS   40   // Both solenoids push — full plunger travel
#define SOLENOID_RELEASE_MS  35   // Spring return time after de-energise
#define DEAD_TIME_MS         15   // Stagger between A and B on engage

// ── Ratchet state tracking ───────────────────────────────────────────────────
// Track ratchet state explicitly to avoid redundant solenoid pulses
// and ensure safe sequencing
enum RatchetState { RATCHET_FREE, RATCHET_ENGAGED, RATCHET_TRANSITIONING };
RatchetState ratchetState = RATCHET_FREE;

// ── TCA9548A I2C multiplexer ─────────────────────────────────────────────────
#define MUX_ADDR      0x70
#define MUX_CH_TOF_F  0
#define MUX_CH_TOF_R  1
#define MUX_CH_IMU    2

// ── Sensor thresholds ────────────────────────────────────────────────────────
#define TOF_ATTACK_MM   350
#define TOF_SCAN_MM     900
#define TOF_CONTACT_MM  120   // Distance at which we consider contact made

// ── Motor control ────────────────────────────────────────────────────────────
#define PWM_MAX         255
#define PWM_SPIN        190
#define PWM_RAM         255
#define ACCEL_STEP_SPIN   8
#define ACCEL_STEP_RAM    6   // Conservative for treads under load
#define ACCEL_STEP_STOP  20

// ── Pivot parameters ─────────────────────────────────────────────────────────
#define PIVOT_DEGREES   180.0f
#define DECEL_ZONE_DEG   45.0f  // Wider than wheel version — accounts for tread inertia
#define GYRO_SCALE     (1.0f / 131.0f)  // MPU-6050 default ±250°/s sensitivity

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
Adafruit_VL53L0X tof;
MPU6050          imu(MPU6050_DEFAULT_ADDRESS, &Wire2);
float            yawAccum    = 0.0f;
uint32_t         tLastUs     = 0;
int16_t          gyroZOffset = 0;

// =============================================================================
// EDGE DETECTION — digitalRead
// Uses the DO pin of each TCRT5000 module.
// WHITE LINE = DO HIGH (potentiometer-set threshold on module).
// If your modules output LOW on white, flip == HIGH to == LOW in both functions.
// =============================================================================
bool edgeDetected() {
  // Read all sensors
  bool fl = digitalRead(PIN_EDGE_FL);
  bool fr = digitalRead(PIN_EDGE_FR);
  bool rl = digitalRead(PIN_EDGE_RL);
  bool rr = digitalRead(PIN_EDGE_RR);
  
  if (fl || fr || rl || rr) {
    delayMicroseconds(500); // Tiny wait to let noise settle
    // Check again - if it's still high, it's a real line
    return (digitalRead(PIN_EDGE_FL) || digitalRead(PIN_EDGE_FR) || 
            digitalRead(PIN_EDGE_RL) || digitalRead(PIN_EDGE_RR));
  }
  return false;
}

// Returns which side triggered the edge — used to choose recovery spin direction.
// -1 = left sensors triggered  →  spin right to face inward
//  1 = right sensors triggered →  spin left to face inward
//  0 = front/rear or both equal → reverse straight
int edgeDirection() {
  bool fl = digitalRead(PIN_EDGE_FL) == HIGH;
  bool fr = digitalRead(PIN_EDGE_FR) == HIGH;
  bool rl = digitalRead(PIN_EDGE_RL) == HIGH;
  bool rr = digitalRead(PIN_EDGE_RR) == HIGH;
  int leftCount  = (fl ? 1 : 0) + (rl ? 1 : 0);
  int rightCount = (fr ? 1 : 0) + (rr ? 1 : 0);
  if (leftCount > rightCount) return -1;
  if (rightCount > leftCount)  return  1;
  return 0;
}

// =============================================================================
// SOLENOID CONTROL — safe two-solenoid ratchet API
// Both solenoids are push-type (extend when energised).
// ENGAGE  : A fires first, B fires after DEAD_TIME_MS — both push pawl into teeth.
// RELEASE : both de-energise simultaneously — spring passively returns pawl.
// No active push is used for release; spring alone clears the mechanism.
// All ratchet operations go through these functions ONLY.
// =============================================================================

// engageRatchet()
// Fires A then B (staggered by DEAD_TIME_MS) to push and hold pawl in teeth.
// Blocks for full plunger travel. Safe to call if already engaged (no-op).
void engageRatchet() {
  if (ratchetState == RATCHET_ENGAGED) return;
  ratchetState = RATCHET_TRANSITIONING;

  digitalWrite(PIN_SOL_A, HIGH);           // primary push — pawl into teeth
  delay(DEAD_TIME_MS);                     // stagger to reduce inrush overlap
  digitalWrite(PIN_SOL_B, HIGH);           // reinforcing push — holds engagement
  delay(SOLENOID_ENGAGE_MS);              // wait for full 10mm plunger travel

  ratchetState = RATCHET_ENGAGED;
  Serial.println("Ratchet: ENGAGED");
}

// releaseRatchet()
// De-energises both solenoids. Spring passively returns pawl to free position.
// MUST complete before any motor reversal.
// Safe to call if already free (no-op).
void releaseRatchet() {
  if (ratchetState == RATCHET_FREE) return;
  ratchetState = RATCHET_TRANSITIONING;

  // Drop both simultaneously — no active push, spring does the work
  digitalWrite(PIN_SOL_A, LOW);
  digitalWrite(PIN_SOL_B, LOW);
  delay(SOLENOID_RELEASE_MS);             // spring return travel time

  ratchetState = RATCHET_FREE;
  Serial.println("Ratchet: RELEASED");
}

// emergencyRelease()
// Identical to releaseRatchet() but skips the state check — use in fault paths.
void emergencyRelease() {
  digitalWrite(PIN_SOL_A, LOW);
  digitalWrite(PIN_SOL_B, LOW);
  ratchetState = RATCHET_FREE;
  Serial.println("Ratchet: EMERGENCY RELEASE");
}

// =============================================================================
// I2C MUX
// =============================================================================
void muxSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire2.beginTransmission(MUX_ADDR);
  Wire2.write(1 << channel);
  Wire2.endTransmission();
  delayMicroseconds(50);  // settle time
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
  if (measure.RangeStatus != 4) {  // 4 = out of range / no target
    return constrain(measure.RangeMilliMeter, 0, 2000);
  }
  return 2000;  // return max if no valid reading
}

// =============================================================================
// MOTOR CONTROL
// =============================================================================
void trapRamp(int& current, int target, int step) {
  if (current < target) current = min(current + step, target);
  else if (current > target) current = max(current - step, target);
}

void setMotors(int l, int r) {
  l = constrain(l, -PWM_MAX, PWM_MAX);
  r = constrain(r, -PWM_MAX, PWM_MAX);
  digitalWrite(PIN_DIR_L, l >= 0 ? LOW : HIGH);
  digitalWrite(PIN_DIR_R, r >= 0 ? LOW : HIGH);
  analogWrite(PIN_MOTOR_L, abs(l));
  analogWrite(PIN_MOTOR_R, abs(r));
}

// =============================================================================
// GYRO
// =============================================================================

// calibrateGyro()
// Averages 500 samples at rest to find zero offset.
// Robot must be stationary during this call.
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

// headingCorrection()
// Reads gyro Z rate and returns a differential PWM correction value.
// Positive = drifting right → reduce right motor, increase left.
// Gain of 0.4f: increase toward 0.8f if robot veers, decrease toward 0.2f
// if it oscillates during ram.
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
// Ratchet is guaranteed FREE before spin — engaging during a counter-rotate
// would lock one tread asymmetrically and damage the mechanism.
// =============================================================================
void executePivot() {
  Serial.println("STATE: PIVOT");

  // Guarantee ratchet is free before any spin
  releaseRatchet();

  muxSelect(MUX_CH_IMU);
  yawAccum = 0.0f;
  tLastUs  = micros();
  int spinPWM = 0;

  while (abs(yawAccum) < PIVOT_DEGREES) {

    // Edge abort during pivot — edgeDetected() uses digitalRead
    if (edgeDetected()) {
      trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      // Ratchet already free — no release needed
      state = STATE_RECOVER;
      return;
    }

    // Gyro integration
    uint32_t now = micros();
    float dt = (now - tLastUs) / 1e6f;
    tLastUs  = now;

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rateZ = (gz - gyroZOffset) * GYRO_SCALE;
    yawAccum += rateZ * dt;

    // Trapezoidal speed — decel in final DECEL_ZONE_DEG degrees
    // Wider zone (45°) than wheels (30°) accounts for tread rotational inertia
    float remaining = PIVOT_DEGREES - abs(yawAccum);
    int targetSpd;
    if (remaining < DECEL_ZONE_DEG) {
      targetSpd = (int)map((long)remaining, 5, (long)DECEL_ZONE_DEG, 40, PWM_SPIN);
      targetSpd = max(targetSpd, 40);  // minimum 40 to keep moving
    } else {
      targetSpd = PWM_SPIN;
    }

    trapRamp(spinPWM, targetSpd, ACCEL_STEP_SPIN);
    setMotors(spinPWM, -spinPWM);  // counter-rotate treads

    // Opportunistic target check in second half of pivot
    // If opponent detected before 180° complete, abort early and ram
    if (remaining < 90.0f) {
      uint16_t dist = readToF(MUX_CH_TOF_F);
      muxSelect(MUX_CH_IMU);  // reselect IMU after ToF read
      if (dist < TOF_ATTACK_MM) {
        Serial.println("Target mid-pivot — transitioning to RAM");
        trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
        setMotors(0, 0);
        state = STATE_RAM;
        return;
      }
    }
  }

  // Pivot complete — stop and transition to RAM
  trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
  setMotors(0, 0);
  delay(30);  // brief settle
  Serial.print("Pivot complete. Yaw: ");
  Serial.println(yawAccum);
  state = STATE_RAM;
}

// =============================================================================
// STATE: SCAN
// Slow spin to locate opponent after pivot or contact loss.
// Ratchet stays FREE — may need to spin any direction.
// =============================================================================
void executeScan() {
  Serial.println("STATE: SCAN");

  // Ensure ratchet is free during scan
  releaseRatchet();

  int scanPWM = 0;
  uint32_t scanStart = millis();
  const uint32_t SCAN_TIMEOUT_MS = 1500;

  while (millis() - scanStart < SCAN_TIMEOUT_MS) {

    // Edge abort — digitalRead via edgeDetected()
    if (edgeDetected()) {
      trapRamp(scanPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    trapRamp(scanPWM, 120, ACCEL_STEP_SPIN);
    setMotors(scanPWM, -scanPWM);

    uint16_t dist = readToF(MUX_CH_TOF_F);
    if (dist < TOF_SCAN_MM) {
      Serial.print("Target at ");
      Serial.print(dist);
      Serial.println(" mm — transitioning to RAM");
      trapRamp(scanPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      state = STATE_RAM;
      return;
    }
  }

  // Timeout — state stays SCAN, loop() will call again
  Serial.println("Scan timeout — continuing");
}

// =============================================================================
// STATE: RAM
// Full-speed attack with trapezoidal ramp and IMU heading hold.
// Engages ratchet on contact, releases before any direction change.
//
// Critical solenoid ordering:
//   Edge detected  → releaseRatchet() FIRST, then stop motors
//   Contact lost   → releaseRatchet() FIRST, then transition to SCAN
//   Motors must NEVER reverse while ratchet is ENGAGED.
// =============================================================================
void executeRam() {
  Serial.println("STATE: RAM");
  int ramPWM = 0;

  while (true) {

    // Highest priority: edge abort — digitalRead via edgeDetected()
    if (edgeDetected()) {
      releaseRatchet();  // blocks ~90ms — pawl must physically clear before reverse
      trapRamp(ramPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    // Trapezoidal ramp to full speed
    // ACCEL_STEP_RAM = 6 (slower than spin) — maximises torque under tread load
    trapRamp(ramPWM, PWM_RAM, ACCEL_STEP_RAM);

    // IMU heading correction — keeps treads straight during ram
    float correction = headingCorrection();
    int corrL = constrain((int)(ramPWM - correction), 0, PWM_MAX);
    int corrR = constrain((int)(ramPWM + correction), 0, PWM_MAX);
    setMotors(corrL, corrR);

    // ToF contact management
    uint16_t dist = readToF(MUX_CH_TOF_F);

    if (dist < TOF_CONTACT_MM && ratchetState == RATCHET_FREE) {
      // Contact made — lock ratchet to hold pushing position.
      // Motors hold last PWM during ~55ms blocking engage call — fine because
      // physical contact is already made; solenoid locks the pushing position.
      engageRatchet();
    }

    if (dist > TOF_SCAN_MM) {
      // Opponent escaped — release ratchet before any direction change
      Serial.println("Contact lost — releasing ratchet");
      releaseRatchet();  // blocks ~90ms — safe to change direction after this
      trapRamp(ramPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      state = STATE_SCAN;
      return;
    }
  }
}

// =============================================================================
// STATE: RECOVER
// Called after edge detection from any state.
// Ratchet is guaranteed FREE on entry (calling state releases first).
// Reverse away from boundary, pivot 90° via IMU, return to SCAN.
// =============================================================================
void executeRecover() {
  Serial.println("STATE: RECOVER");

  // Safety guarantee — no-op if already free
  releaseRatchet();

  // Capture edge direction before reversing clears the sensors
  int dir = edgeDirection();

  int recPWM = 0;

  // Reverse away from boundary
  uint32_t tReverse = millis();
  while (millis() - tReverse < 350) {
    trapRamp(recPWM, 150, ACCEL_STEP_STOP);
    setMotors(-recPWM, -recPWM);
  }

  trapRamp(recPWM, 0, ACCEL_STEP_STOP);
  setMotors(0, 0);
  delay(20);

  // 90° recovery pivot using IMU
  muxSelect(MUX_CH_IMU);
  yawAccum = 0.0f;
  tLastUs  = micros();
  int pivPWM = 0;

  while (abs(yawAccum) < 90.0f) {

    // Edge check during recovery pivot — prevents spinning into a second boundary
    if (edgeDetected()) {
      trapRamp(pivPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      state = STATE_RECOVER;  // re-enter with fresh direction reading
      return;
    }

    uint32_t now = micros();
    float dt = (now - tLastUs) / 1e6f;
    tLastUs  = now;

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rateZ = (gz - gyroZOffset) * GYRO_SCALE;
    yawAccum += rateZ * dt;

    // Decel in last 20° of recovery pivot
    float rem = 90.0f - abs(yawAccum);
    int tgt = (rem < 20.0f) ? 50 : 130;
    trapRamp(pivPWM, tgt, 6);

    // Spin away from the edge that triggered
    if (dir >= 0) setMotors(-pivPWM,  pivPWM);  // spin left
    else          setMotors( pivPWM, -pivPWM);   // spin right
  }

  trapRamp(pivPWM, 0, ACCEL_STEP_STOP);
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

  // ── Solenoid pins — set LOW before anything else ──────────────────────────
  // Both push-type: LOW = de-energised = spring holds pawl free.
  // 10kΩ pulldown on each MOSFET gate covers boot float, but explicit LOW
  // here guarantees correct state from the very first instruction.
  pinMode(PIN_SOL_A, OUTPUT);
  pinMode(PIN_SOL_B, OUTPUT);
  digitalWrite(PIN_SOL_A, LOW);
  digitalWrite(PIN_SOL_B, LOW);
  ratchetState = RATCHET_FREE;

  // ── Edge sensor pins — INPUT_PULLDOWN ─────────────────────────────────────
  // TCRT5000 DO pin is active-HIGH driven by onboard comparator.
  // INPUT_PULLDOWN ensures defined LOW if a sensor is disconnected,
  // preventing a floating pin from triggering a false edge detection.
  pinMode(PIN_EDGE_FL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_FR, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RR, INPUT_PULLDOWN);

  // ── Other pins ────────────────────────────────────────────────────────────
  pinMode(PIN_START,   INPUT_PULLDOWN);
  pinMode(PIN_DIR_L,   OUTPUT);
  pinMode(PIN_DIR_R,   OUTPUT);
  pinMode(PIN_MOTOR_L, OUTPUT);
  pinMode(PIN_MOTOR_R, OUTPUT);
  setMotors(0, 0);

  // ── I2C on Wire2 (pins 24=SCL2, 25=SDA2) ─────────────────────────────────
  // Note: pins 40/41 are not hardware I2C capable on Teensy 4.1.
  // Wire2 is the closest native hardware bus to the requested pins.
  Wire2.begin();
  Wire2.setClock(400000);  // 400 kHz fast-mode — VL53L0X and MPU-6050 both support this

  // ── TCA9548A mux ──────────────────────────────────────────────────────────
  muxDisableAll();

  // ── VL53L0X (mux ch0) ────────────────────────────────────────────────────
  muxSelect(MUX_CH_TOF_F);
  if (!tof.begin(VL53L0X_I2C_ADDR, false, &Wire2)) {
    Serial.println("ERROR: VL53L0X not found!");
    while (1);
  }
  tof.setMeasurementTimingBudgetMicroSeconds(20000);  // 20ms — fast mode
  tof.startRangeContinuous();
  Serial.println("VL53L0X OK");

  // ── MPU-6050 (mux ch2) ───────────────────────────────────────────────────
  muxSelect(MUX_CH_IMU);
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("ERROR: MPU-6050 not found!");
    while (1);
  }
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250°/s for precision pivots
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g for contact detection
  Serial.println("MPU-6050 OK");

  // ── Gyro calibration — robot must be stationary ───────────────────────────
  Serial.println("Calibrating gyro — hold still...");
  calibrateGyro();

  // ── Edge sensor sanity check ──────────────────────────────────────────────
  // All four should read 0 on the dark dohyo surface at startup.
  // If any read 1, the pot threshold is too sensitive or sensor is misaligned.
  Serial.println("Edge sensor check (all should be 0 on dark surface):");
  Serial.print("  FL="); Serial.print(digitalRead(PIN_EDGE_FL));
  Serial.print("  FR="); Serial.print(digitalRead(PIN_EDGE_FR));
  Serial.print("  RL="); Serial.print(digitalRead(PIN_EDGE_RL));
  Serial.print("  RR="); Serial.println(digitalRead(PIN_EDGE_RR));

  // ── Solenoid self-test ────────────────────────────────────────────────────
  // Fires both solenoids sequentially. Listen for two distinct clicks.
  // Comment out for competition if startup delay is a concern.
  Serial.println("Solenoid self-test...");
  engageRatchet();
  delay(200);
  releaseRatchet();
  Serial.println("Solenoid self-test complete.");

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
        delay(5000);  // mandatory wait — adjust per ruleset
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
      emergencyRelease();
      setMotors(0, 0);
      break;
  }
}