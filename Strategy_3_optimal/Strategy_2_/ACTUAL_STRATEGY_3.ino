#include <ESP32Servo.h>

// ── Pin Assignments ───────────────────────────────────────────────────────────
#define PIN_EDGE_FL  14
#define PIN_EDGE_FR  15
#define PIN_EDGE_RL  16
#define PIN_EDGE_RR  17
#define PIN_START        13   // Start module
#define PIN_MOTOR_L      11   // Motor 1 PWM (left tread)
#define PIN_MOTOR_R      12   // Motor 2 PWM (right tread)

// ── Tuning Constants ──────────────────────────────────────────────────────────
#define PWM_MAX          255
#define PWM_SPIN         190
#define PWM_RAM          255
#define ACCEL_STEP_SPIN    8
#define ACCEL_STEP_RAM     6
#define ACCEL_STEP_STOP   20

// ── State Machine ─────────────────────────────────────────────────────────────
enum RobotState {
  STATE_WAIT,
  STATE_DRIVE,
  STATE_RECOVER,
  STATE_HALT
};

// ── Globals ───────────────────────────────────────────────────────────────────
RobotState state = STATE_WAIT;
Servo      motorL, motorR;

volatile bool haltRequested  = false;
volatile bool startRequested = false;

// ── ISR ───────────────────────────────────────────────────────────────────────
void IRAM_ATTR ISR_startStop() {
  if (digitalRead(PIN_START) == HIGH) startRequested = true;
  else                                haltRequested  = true;
}

// ── Motor Control ─────────────────────────────────────────────────────────────
void trapRamp(int &current, int target, int step) {
  if      (current < target) current = min(current + step, target);
  else if (current > target) current = max(current - step, target);
}

// Positive = forward, negative = reverse.
// Right motor inverted to match physical mounting orientation.
void setMotors(int l, int r) {
  l = constrain(l, -PWM_MAX, PWM_MAX);
  r = constrain(r, -PWM_MAX, PWM_MAX);
  motorL.writeMicroseconds(map(l, -PWM_MAX, PWM_MAX, 1000, 2000));
  motorR.writeMicroseconds(map(r, -PWM_MAX, PWM_MAX, 2000, 1000)); // inverted
}

// ── Edge Detection ────────────────────────────────────────────────────────────
bool edgeDetected() {
  if (digitalRead(PIN_EDGE_FL) || digitalRead(PIN_EDGE_FR) ||
      digitalRead(PIN_EDGE_RL) || digitalRead(PIN_EDGE_RR)) {
    delayMicroseconds(500);
    return (digitalRead(PIN_EDGE_FL) || digitalRead(PIN_EDGE_FR) ||
            digitalRead(PIN_EDGE_RL) || digitalRead(PIN_EDGE_RR));
  }
  return false;
}

// -1 = left triggered  → spin right to face inward
//  1 = right triggered → spin left to face inward
//  0 = front/rear/equal → treat as centre, spin left
int edgeDirection() {
  delayMicroseconds(500);
  bool fl = digitalRead(PIN_EDGE_FL) == HIGH;
  bool fr = digitalRead(PIN_EDGE_FR) == HIGH;
  bool rl = digitalRead(PIN_EDGE_RL) == HIGH;
  bool rr = digitalRead(PIN_EDGE_RR) == HIGH;

  int leftCount  = (fl ? 1 : 0) + (rl ? 1 : 0);
  int rightCount = (fr ? 1 : 0) + (rr ? 1 : 0);
  if (leftCount  > rightCount) return -1;
  if (rightCount > leftCount)  return  1;
  if (fl && rr && !fr && !rl)  return -1;
  if (fr && rl && !fl && !rr)  return  1;
  return 0;
}

// ── Spin helper ───────────────────────────────────────────────────────────────
// Spins in direction dir, ramping down cleanly before returning.
// Returns false if halt was requested mid-spin.
bool spinRampDown(int &pivPWM, int dir) {
  while (pivPWM > 0) {
    if (haltRequested) { setMotors(0, 0); return false; }
    trapRamp(pivPWM, 0, ACCEL_STEP_STOP);
    if (dir >= 0) setMotors(-pivPWM,  pivPWM);  // right hit → spin left
    else          setMotors( pivPWM, -pivPWM);  // left hit  → spin right
  }
  return true;
}

// =============================================================================
// STATE: DRIVE
// Full speed forward. Transitions to RECOVER on edge detection.
// =============================================================================
void executeDrive() {
  Serial.println("STATE: DRIVE");
  int drivePWM = 0;

  while (true) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }

    if (edgeDetected()) {
      // Ramp down before handing off to RECOVER
      while (drivePWM > 0) {
        if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
        trapRamp(drivePWM, 0, ACCEL_STEP_STOP);
        setMotors(drivePWM, -drivePWM);
      }
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    trapRamp(drivePWM, PWM_RAM, ACCEL_STEP_RAM);
    setMotors(drivePWM, -drivePWM);  // right motor inverted in setMotors
  }
}

// =============================================================================
// STATE: RECOVER
// Reverse 350ms away from boundary, then timed 90° pivot away from the
// triggering side. Edge detection active during pivot — re-enters RECOVER
// with a fresh direction read if a second boundary is hit.
// =============================================================================
void executeRecover() {
  Serial.println("STATE: RECOVER");

  // Read direction BEFORE reversing — sensors clear once we back up
  int dir = edgeDirection();
  Serial.printf("  Edge direction: %s\n",
    dir < 0 ? "LEFT (spinning right)" :
    dir > 0 ? "RIGHT (spinning left)" : "CENTRE (spinning left)");

  // ── Reverse away from edge ─────────────────────────────────────────────────
  int      recPWM   = 0;
  uint32_t tReverse = millis();
  while (millis() - tReverse < 350) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
    trapRamp(recPWM, 150, ACCEL_STEP_STOP);
    setMotors(-recPWM, recPWM);  // both reverse
  }
  // Ramp down reverse
  while (recPWM > 0) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }
    trapRamp(recPWM, 0, ACCEL_STEP_STOP);
    setMotors(-recPWM, recPWM);
  }
  setMotors(0, 0);
  delay(20);

  // ── Timed 90° pivot ────────────────────────────────────────────────────────
  // 400ms at PWM_SPIN — tune this value if turn is too short or too long
  int      pivPWM = 0;
  uint32_t tPivot = millis();
  while (millis() - tPivot < 400) {
    if (haltRequested) { setMotors(0, 0); state = STATE_HALT; return; }

    if (edgeDetected()) {
      // Second boundary hit — ramp down cleanly then re-enter RECOVER
      if (!spinRampDown(pivPWM, dir)) { state = STATE_HALT; return; }
      setMotors(0, 0);
      state = STATE_RECOVER;
      return;
    }

    trapRamp(pivPWM, PWM_SPIN, ACCEL_STEP_SPIN);
    if (dir >= 0) setMotors(-pivPWM,  pivPWM);  // right/centre → spin left
    else          setMotors( pivPWM, -pivPWM);  // left → spin right
  }

  // Ramp down at end of pivot timer
  if (!spinRampDown(pivPWM, dir)) { state = STATE_HALT; return; }
  setMotors(0, 0);
  delay(20);

  state = STATE_DRIVE;
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  // Edge sensors — INPUT_PULLDOWN: reads LOW when off the white line
  pinMode(PIN_EDGE_FL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_FR, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RR, INPUT_PULLDOWN);

  // Start module
  pinMode(PIN_START, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_START), ISR_startStop, CHANGE);

  // Motors — ESP32Servo allocates a hardware timer per instance
  motorL.attach(PIN_MOTOR_L, 1000, 2000);
  motorR.attach(PIN_MOTOR_R, 1000, 2000);
  setMotors(0, 0);  // 1500µs neutral — begins ESC arming
  delay(2000);

  Serial.println("========================================");
  Serial.println("         SUMOBOT — EDGE ONLY");
  Serial.println("========================================");
  Serial.println("Edge sensor check (all should be 0 on dark surface):");
  Serial.printf("  FL=%d  FR=%d  RL=%d  RR=%d\n",
    digitalRead(PIN_EDGE_FL), digitalRead(PIN_EDGE_FR),
    digitalRead(PIN_EDGE_RL), digitalRead(PIN_EDGE_RR));
  Serial.println("Ready. Waiting for start signal...");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  if (haltRequested) {
    setMotors(0, 0);
    state = STATE_HALT;
    haltRequested = false;
  }

  switch (state) {
    case STATE_WAIT:
      if (startRequested) {
        startRequested = false;
        Serial.println("Start!");
        state = STATE_DRIVE;
      }
      break;

    case STATE_DRIVE:
      executeDrive();
      break;

    case STATE_RECOVER:
      executeRecover();
      break;

    case STATE_HALT:
    default:
      setMotors(0, 0);
      if (startRequested) {
        startRequested = false;
        haltRequested  = false;
        Serial.println("Re-armed — waiting for start signal");
        state = STATE_WAIT;
      }
      break;
  }
}