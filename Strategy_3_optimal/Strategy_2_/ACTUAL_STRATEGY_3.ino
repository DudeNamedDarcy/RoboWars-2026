// =============================================================================
// IMU-PIVOT RAM — Full Implementation
// Hardware: PJRC Teensy 4.1 (ARM Cortex-M7 @ 600 MHz)
// =============================================================================
//
// PIN ASSIGNMENT SUMMARY
// ─────────────────────────────────────────────────────────────────────────────
//  Pin  4  → Ratchet solenoid (HIGH = engage, LOW = release)
//  Pin  5  → Edge sensor FL (TCRT5000 analog — white = HIGH > EDGE_THRESHOLD)
//  Pin  6  → Edge sensor FR (TCRT5000 analog)
//  Pin  7  → Edge sensor RL (TCRT5000 analog)
//  Pin  8  → Edge sensor RR (TCRT5000 analog)
//  Pin  9  → Start module (HIGH = match start)
//  Pin 10  → Motor 1 PWM  (left tread)
//  Pin 11  → Motor 2 PWM  (right tread)
//  Pin 24  → SCL2  (Wire2 — hardware I2C bus 2)  ← closest safe substitute
//  Pin 25  → SDA2  (Wire2 — hardware I2C bus 2)    for requested pins 40/41
//
// ─────────────────────────────────────────────────────────────────────────────
// NOTE ON PINS 40 / 41:
//   Pins 40 and 41 on the Teensy 4.1 are FlexPWM GPIO — they are NOT part
//   of any hardware I2C peripheral. The three hardware I2C buses are:
//     Wire  → pins 18 (SDA)  / 19 (SCL)
//     Wire1 → pins 17 (SDA)  / 16 (SCL)   or  25 (SDA) / 24 (SCL)
//     Wire2 → pins 25 (SDA2) / 24 (SCL2)
//   Using pins 40/41 for I2C would require software bit-bang I2C, which
//   breaks VL53L0X and MPU-6050 library compatibility.
//   Implementation uses Wire2 (pins 24/25) — change the Wire2.begin() call
//   to a SoftWire instance if you require 40/41 at the cost of speed.
//
// I2C DEVICE MAP (via TCA9548A mux on Wire2)
//   Mux channel 0 → VL53L0X  (front ToF distance)
//   Mux channel 1 → VL53L0X  (optional: rear/side ToF)
//   Mux channel 2 → MPU-6050 (IMU — gyro Z for pivot)
//   All devices at their default addresses (mux separates address collisions)
//
// TREAD NOTES:
//   Tank treads have higher rotational inertia than wheels.
//   The DECEL_ZONE_DEG constant (degrees before target at which we start
//   slowing the spin) is set to 45° — wider than the wheel version (30°).
//   Tune DECEL_ZONE_DEG between 35–55° based on your tread tension & weight.
//   The trapezoidal accel step during the RAM phase is set to 6 (vs 8 for
//   wheels) to avoid tread slip under full load.
//
// LIBRARIES REQUIRED (install via Arduino Library Manager):
//   - Adafruit_VL53L0X  by Adafruit
//   - MPU6050           by Electronic Cats  (or jrowberg/i2cdevlib)
//   - Wire              (bundled with Teensyduino)
// =============================================================================

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define PIN_RATCHET     4
#define PIN_EDGE_FL     5
#define PIN_EDGE_FR     6
#define PIN_EDGE_RL     7
#define PIN_EDGE_RR     8
#define PIN_START       9
#define PIN_MOTOR_L    10   // left tread  (PWM)
#define PIN_MOTOR_R    11   // right tread (PWM)
// I2C on Wire2: SCL=24, SDA=25 (hardware bus — see note above)

// ── TCA9548A I2C multiplexer ─────────────────────────────────────────────────
#define MUX_ADDR        0x70
#define MUX_CH_TOF_F    0    // VL53L0X front
#define MUX_CH_TOF_R    1    // VL53L0X rear/side (optional)
#define MUX_CH_IMU      2    // MPU-6050

// ── Sensor thresholds ────────────────────────────────────────────────────────
#define EDGE_THRESHOLD   1   // TCRT5000 analog value — white line reads HIGH, probably change? Originally was 600
                               // Calibrate: dark surface ~200, white >700
#define TOF_ATTACK_MM    350   // Start full ram when opponent < this distance
#define TOF_SCAN_MM      900   // Max detection range during scan

// ── Motor control ────────────────────────────────────────────────────────────
#define PWM_MAX          255
#define PWM_SPIN         190   // Spin speed during pivot (lower = more control)
#define PWM_APPROACH     160   // Speed during initial approach/scan
#define PWM_RAM          255   // Full ram speed
#define ACCEL_STEP_SPIN    8   // Trapezoidal step for spin phase
#define ACCEL_STEP_RAM     6   // Trapezoidal step for ram phase (treads: lower)
#define ACCEL_STEP_STOP   20   // Decel step (faster stop)

// ── Pivot parameters ─────────────────────────────────────────────────────────
#define PIVOT_DEGREES   180.0f  // Target rotation for pivot
#define DECEL_ZONE_DEG   45.0f  // Begin decelerating this many degrees before target
                                // Wider than wheel version (30°) — accounts for
                                // higher tread rotational inertia
#define GYRO_SCALE    (1.0f / 131.0f)  // MPU-6050 default ±250°/s sensitivity

// ── State machine ────────────────────────────────────────────────────────────
enum RobotState {
  STATE_WAIT,        // Waiting for start signal
  STATE_SCAN,        // Spinning in place to locate opponent pre-pivot
  STATE_PIVOT,       // Executing 180° IMU-timed pivot
  STATE_RAM,         // Full-speed attack run
  STATE_RECOVER,     // Edge detected — reverse and reacquire
  STATE_HALT         // Match over / fault
};

// ── Globals ──────────────────────────────────────────────────────────────────
RobotState      state          = STATE_WAIT;
Adafruit_VL53L0X tof;
MPU6050         imu;

int  leftPWM   = 0;
int  rightPWM  = 0;

// Gyro integration
float    yawAccum  = 0.0f;
uint32_t tLastUs   = 0;
int16_t  gyroZOffset = 0;   // Calibrated zero offset

// Edge debounce
bool     edgeActive   = false;
uint32_t edgeTimeMs   = 0;
#define  EDGE_DEBOUNCE_MS  10

// =============================================================================
// I2C MUX helper
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
// ToF read (selects mux channel first)
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
// Edge detection (analog TCRT5000)
// Returns true if ANY sensor sees white (boundary)
// =============================================================================
bool edgeDetected() {
  // Assuming HIGH means white line is detected (need to test this)
  bool fl = digitalRead(PIN_EDGE_FL) == HIGH;
  bool fr = digitalRead(PIN_EDGE_FR) == HIGH;
  bool rl = digitalRead(PIN_EDGE_RL) == HIGH;
  bool rr = digitalRead(PIN_EDGE_RR) == HIGH;
  return (fl || fr || rl || rr);
}

// Which side triggered? Used to determine recovery direction
// Returns -1 = left, 1 = right, 0 = front/rear (reverse)
int edgeDirection() {
  bool fl = analogRead(PIN_EDGE_FL) > EDGE_THRESHOLD;
  bool fr = analogRead(PIN_EDGE_FR) > EDGE_THRESHOLD;
  bool rl = analogRead(PIN_EDGE_RL) > EDGE_THRESHOLD;
  bool rr = analogRead(PIN_EDGE_RR) > EDGE_THRESHOLD;
  int leftCount  = (fl ? 1 : 0) + (rl ? 1 : 0);
  int rightCount = (fr ? 1 : 0) + (rr ? 1 : 0);
  if (leftCount > rightCount) return -1;
  if (rightCount > leftCount)  return  1;
  return 0;
}

// =============================================================================
// Trapezoidal ramp — call every loop tick
// Increments/decrements 'current' toward 'target' by 'step'
// Then applies to both motors at the given l/r ratio
// =============================================================================
void trapRamp(int& current, int target, int step) {
  if (current < target) current = min(current + step, target);
  else if (current > target) current = max(current - step, target);
}

// Apply PWM to both motors — positive = forward, negative = reverse
// For tank treads: both tracks same direction = straight, opposite = spin
void setMotors(int l, int r) {
  l = constrain(l, -PWM_MAX, PWM_MAX);
  r = constrain(r, -PWM_MAX, PWM_MAX);

  // Motor driver assumes PWM + direction pin. Adapt to your driver below.
  // Example: L298N or similar with separate DIR and PWM pins.
  // If using a motor driver that takes signed PWM natively, simplify this.
  analogWrite(PIN_MOTOR_L, abs(l));
  // If your driver uses a separate direction pin, set it here:
  // digitalWrite(PIN_DIR_L, l >= 0 ? HIGH : LOW);
  analogWrite(PIN_MOTOR_R, abs(r));
  // digitalWrite(PIN_DIR_R, r >= 0 ? HIGH : LOW);
}

// =============================================================================
// Gyro calibration — average 500 samples at rest to find zero offset
// Call before match start, robot must be stationary
// =============================================================================
void calibrateGyro() {
  muxSelect(MUX_CH_IMU);
  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroZOffset = (int16_t)(sum / samples);
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZOffset);
}

// =============================================================================
// IMU yaw heading correction during straight-line ram
// Reads gyro Z, returns differential correction (positive = turn right)
// =============================================================================
float headingCorrection() {
  muxSelect(MUX_CH_IMU);
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float rateZ = (gz - gyroZOffset) * GYRO_SCALE;  // °/s
  // Small proportional correction: if drifting right (pos rate), steer left
  return rateZ * 0.4f;  // tune 0.2–0.8 based on tread responsiveness
}

// =============================================================================
// STATE: PIVOT
// Executes a 180° counter-rotating pivot using gyro Z integration
// Left tread forward, right tread reverse (pivots clockwise)
// Decelerates in the last DECEL_ZONE_DEG degrees to avoid overshoot
// =============================================================================
void executePivot() {
  Serial.println("STATE: PIVOT");
  muxSelect(MUX_CH_IMU);

  yawAccum = 0.0f;
  tLastUs  = micros();
  int spinPWM = 0;

  while (abs(yawAccum) < PIVOT_DEGREES) {
    // ── Edge abort during pivot ──────────────────────────────────────────────
    if (edgeDetected()) {
      setMotors(0, 0);
      digitalWrite(PIN_RATCHET, LOW);
      state = STATE_RECOVER;
      return;
    }

    // ── Gyro integration ─────────────────────────────────────────────────────
    uint32_t now = micros();
    float dt = (now - tLastUs) / 1e6f;
    tLastUs  = now;

    // Read IMU on mux ch2
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rateZ = (gz - gyroZOffset) * GYRO_SCALE;  // °/s
    yawAccum += rateZ * dt;

    // ── Trapezoidal speed control ─────────────────────────────────────────────
    float remaining = PIVOT_DEGREES - abs(yawAccum);
    int targetSpd;

    if (remaining < DECEL_ZONE_DEG) {
      // Decel zone — ramp down. Treads need more room to stop than wheels.
      // Map remaining degrees [DECEL_ZONE_DEG..5] to [PWM_SPIN..40]
      targetSpd = (int)map((long)remaining, 5, (long)DECEL_ZONE_DEG,
                            40, PWM_SPIN);
      targetSpd = max(targetSpd, 40);  // minimum 40 to keep moving
    } else {
      targetSpd = PWM_SPIN;
    }

    trapRamp(spinPWM, targetSpd, ACCEL_STEP_SPIN);

    // Counter-rotate: L forward, R reverse → clockwise spin
    setMotors(spinPWM, -spinPWM);

    // ── Opportunistic target check during pivot ───────────────────────────────
    // If opponent detected during pivot, abort early and ram
    if (remaining < 90.0f) {  // only check in second half of pivot
      uint16_t dist = readToF(MUX_CH_TOF_F);
      muxSelect(MUX_CH_IMU);  // reselect IMU after ToF read
      if (dist < TOF_ATTACK_MM) {
        Serial.println("Target acquired mid-pivot — aborting to RAM");
        // Hard stop pivot
        trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
        setMotors(spinPWM, -spinPWM);
        state = STATE_RAM;
        return;
      }
    }
  }

  // Pivot complete — stop and transition
  trapRamp(spinPWM, 0, ACCEL_STEP_STOP);
  setMotors(spinPWM, -spinPWM);
  delay(30);  // brief settle
  setMotors(0, 0);

  Serial.print("Pivot complete. Total yaw: ");
  Serial.println(yawAccum);
  state = STATE_RAM;
}

// =============================================================================
// STATE: SCAN
// Short spin-scan if no opponent detected after pivot
// Sweeps ±45° looking for target, then transitions to RAM or continues scan
// =============================================================================
void executeScan() {
  Serial.println("STATE: SCAN");
  int scanPWM = 0;
  uint32_t scanStart = millis();
  const uint32_t SCAN_TIMEOUT_MS = 1500;

  // Slow clockwise spin
  while (millis() - scanStart < SCAN_TIMEOUT_MS) {
    if (edgeDetected()) {
      state = STATE_RECOVER;
      return;
    }

    trapRamp(scanPWM, 120, ACCEL_STEP_SPIN);
    setMotors(scanPWM, -scanPWM);

    uint16_t dist = readToF(MUX_CH_TOF_F);
    if (dist < TOF_SCAN_MM) {
      Serial.print("Target found at ");
      Serial.print(dist);
      Serial.println(" mm — transitioning to RAM");
      trapRamp(scanPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      state = STATE_RAM;
      return;
    }
  }

  // No target found — re-pivot 90° and try again
  Serial.println("Scan timeout — re-pivoting 90°");
  // (simplified: just continue scan loop in practice)
}

// =============================================================================
// STATE: RAM
// Full-speed attack with trapezoidal ramp and IMU heading hold
// Monitors ToF for contact, edge for boundary
// =============================================================================
void executeRam() {
  Serial.println("STATE: RAM");
  int ramPWM = 0;

  while (true) {
    // ── Highest priority: edge abort ─────────────────────────────────────────
    if (edgeDetected()) {
      digitalWrite(PIN_RATCHET, LOW);
      trapRamp(ramPWM, 0, ACCEL_STEP_STOP);
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    // ── Trapezoidal ramp to full speed ───────────────────────────────────────
    trapRamp(ramPWM, PWM_RAM, ACCEL_STEP_RAM);

    // ── IMU heading correction ───────────────────────────────────────────────
    float correction = headingCorrection();  // ±PWM units
    int corrL = constrain((int)(ramPWM - correction), 0, PWM_MAX);
    int corrR = constrain((int)(ramPWM + correction), 0, PWM_MAX);
    setMotors(corrL, corrR);

    // ── Check if opponent still detected ─────────────────────────────────────
    uint16_t dist = readToF(MUX_CH_TOF_F);
    if (dist > TOF_SCAN_MM) {
      // Lost contact — decel and return to scan
      Serial.println("Contact lost — returning to SCAN");
      trapRamp(ramPWM, 0, ACCEL_STEP_STOP);
      setMotors(ramPWM, ramPWM);
      state = STATE_SCAN;
      return;
    }

    // ── Engage ratchet when close (optional — for heavy push lock) ───────────
    if (dist < 120) {
      digitalWrite(PIN_RATCHET, HIGH);
    }
  }
}

// =============================================================================
// STATE: RECOVER
// Back away from edge, pivot away, re-enter scan/ram
// =============================================================================
void executeRecover() {
  Serial.println("STATE: RECOVER");
  digitalWrite(PIN_RATCHET, LOW);  // always release ratchet before reverse

  int dir = edgeDirection();  // -1=left, 0=front/rear, 1=right
  int recPWM = 0;

  // Reverse away from edge
  uint32_t tReverse = millis();
  while (millis() - tReverse < 350) {
    trapRamp(recPWM, 150, ACCEL_STEP_STOP);
    setMotors(-recPWM, -recPWM);
  }

  // Spin away from the edge direction
  trapRamp(recPWM, 0, ACCEL_STEP_STOP);
  setMotors(0, 0);
  delay(20);

  // 90° pivot using IMU
  yawAccum = 0.0f;
  tLastUs  = micros();
  int pivPWM = 0;
  float targetPivot = 90.0f;

  muxSelect(MUX_CH_IMU);

  while (abs(yawAccum) < targetPivot) {
    uint32_t now = micros();
    float dt = (now - tLastUs) / 1e6f;
    tLastUs  = now;
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rateZ = (gz - gyroZOffset) * GYRO_SCALE;
    yawAccum += rateZ * dt;

    float rem = targetPivot - abs(yawAccum);
    int tgt = (rem < 20.0f) ? 50 : 130;
    trapRamp(pivPWM, tgt, 6);

    // Spin direction based on which edge triggered
    if (dir >= 0) {
      setMotors(-pivPWM, pivPWM);  // spin left
    } else {
      setMotors(pivPWM, -pivPWM);  // spin right
    }
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
  while (!Serial && millis() < 3000);  // wait up to 3s for serial monitor

  // ── Pin modes ─────────────────────────────────────────────────────────────
  pinMode(PIN_RATCHET, OUTPUT);
  pinMode(PIN_START,   INPUT_PULLDOWN);
  pinMode(PIN_MOTOR_L, OUTPUT);
  pinMode(PIN_MOTOR_R, OUTPUT);
  // Edge sensors are analog — no pinMode needed for analogRead

  digitalWrite(PIN_RATCHET, LOW);
  setMotors(0, 0);

  // ── I2C on Wire2 (pins 24=SCL2, 25=SDA2) ─────────────────────────────────
  // Reminder: pins 40/41 are not hardware I2C capable on Teensy 4.1.
  // Wire2 is the closest native bus with accessible pins.
  Wire2.begin();
  Wire2.setClock(400000);  // 400 kHz fast-mode — VL53L0X and MPU-6050 both support this

  // ── TCA9548A mux init ─────────────────────────────────────────────────────
  muxDisableAll();
  Serial.println("TCA9548A mux initialized");

  // ── VL53L0X init (via mux ch0) ───────────────────────────────────────────
  muxSelect(MUX_CH_TOF_F);
  if (!tof.begin(VL53L0X_I2C_ADDR, false, &Wire2)) {
    Serial.println("ERROR: VL53L0X not found on mux ch0!");
    while (1);
  }
  tof.setMeasurementTimingBudgetMicroSeconds(20000);  // 20ms — fast mode
  tof.startRangeContinuous();
  Serial.println("VL53L0X OK");

  // ── MPU-6050 init (via mux ch2) ──────────────────────────────────────────
  muxSelect(MUX_CH_IMU);
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("ERROR: MPU-6050 not found on mux ch2!");
    while (1);
  }
  // Set gyro range to ±250°/s for precision pivots
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  // Set accel range to ±2g for contact detection
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  Serial.println("MPU-6050 OK");

  // ── Gyro calibration (robot must be stationary) ──────────────────────────
  Serial.println("Calibrating gyro — hold robot still...");
  calibrateGyro();
  Serial.println("Calibration complete.");

  Serial.println("Ready. Waiting for start signal on pin 9...");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  switch (state) {

    // ────────────────────────────────────────────────────────────────────────
    case STATE_WAIT:
      if (digitalRead(PIN_START) == HIGH) {
        Serial.println("Start signal received!");
        // 5-second mandatory delay (common rule — adjust or remove per ruleset)
        delay(5000);
        state = STATE_PIVOT;
      }
      break;

    // ────────────────────────────────────────────────────────────────────────
    case STATE_PIVOT:
      executePivot();
      // executePivot() sets state internally (RAM or RECOVER)
      break;

    // ────────────────────────────────────────────────────────────────────────
    case STATE_SCAN:
      executeScan();
      // executeScan() sets state internally (RAM or loops)
      break;

    // ────────────────────────────────────────────────────────────────────────
    case STATE_RAM:
      executeRam();
      // executeRam() sets state internally (SCAN or RECOVER)
      break;

    // ────────────────────────────────────────────────────────────────────────
    case STATE_RECOVER:
      executeRecover();
      // executeRecover() sets state to SCAN
      break;

    // ────────────────────────────────────────────────────────────────────────
    case STATE_HALT:
    default:
      setMotors(0, 0);
      digitalWrite(PIN_RATCHET, LOW);
      break;
  }
}

// =============================================================================
// TUNING GUIDE
// ─────────────────────────────────────────────────────────────────────────────
//
// DECEL_ZONE_DEG (45°)
//   Increase if treads overshoot the 180° target.
//   Decrease if pivot stops too early (hasn't rotated enough).
//   Start at 45°, test in 5° increments.
//
// PWM_SPIN (190)
//   Lower values give more precise pivots at cost of speed.
//   Higher risk of yaw overshoot with tank treads.
//   Recommend 160–200 range.
//
// ACCEL_STEP_RAM (6)
//   Lower = longer ramp, more torque sustained, less wheel/tread slip.
//   Raise to 8–10 if you need faster acceleration (lighter robot).
//
// EDGE_THRESHOLD (600)
//   Calibrate with: Serial.println(analogRead(PIN_EDGE_FL));
//   Set to midpoint between dark-surface reading and white-boundary reading.
//   Typical: dark ~150–250, white ~700–900 → threshold ~600.
//
// headingCorrection() gain (0.4f)
//   If robot veers left/right during ram, increase toward 0.8f.
//   If it oscillates/wiggles, decrease toward 0.2f.
//
// MOTOR DRIVER WIRING NOTE:
//   This code uses analogWrite() for PWM magnitude.
//   If your motor driver (e.g. L298N) uses separate DIR + EN pins,
//   add digitalWrite(PIN_DIR_L, ...) calls in setMotors() above.
//   If using a signed-PWM driver (e.g. TB6612FNG with IN1/IN2 logic),
//   replace the analogWrite calls with the appropriate API.
//
// =============================================================================
