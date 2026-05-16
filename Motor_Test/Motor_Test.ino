#include <Servo.h>

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PIN_EDGE_FL  3
#define PIN_EDGE_FR  4
#define PIN_EDGE_RL  5
#define PIN_EDGE_RR  6
#define PIN_MOTOR_L  8
#define PIN_MOTOR_R  9

// ── Motor ─────────────────────────────────────────────────────────────────────
#define PWM_MAX      255
#define PWM_CREEP    120
#define PWM_REVERSE  180
#define PWM_TURN     160

Servo motorL, motorR;

void setMotors(int l, int r) {
  l = constrain(l, -PWM_MAX, PWM_MAX);
  r = constrain(r, -PWM_MAX, PWM_MAX);
  motorL.writeMicroseconds(map(l, -PWM_MAX, PWM_MAX, 1000, 2000));
  motorR.writeMicroseconds(map(r, -PWM_MAX, PWM_MAX, 1000, 2000));
}

// ── Edge reading ──────────────────────────────────────────────────────────────
uint8_t sampleEdges() {
  uint8_t mask = 0;
  if (digitalRead(PIN_EDGE_FL)) mask |= (1 << 0);  // no !
  if (digitalRead(PIN_EDGE_FR)) mask |= (1 << 1);  // no !
  if (digitalRead(PIN_EDGE_RL)) mask |= (1 << 2);  // no !
  if (digitalRead(PIN_EDGE_RR)) mask |= (1 << 3);  // no !
  return mask;
}

uint8_t readEdgesDebounced() {
  uint8_t first = sampleEdges();
  if (first == 0) return 0;
  delayMicroseconds(500);
  return sampleEdges() & first;
}

// ── Edge responses ────────────────────────────────────────────────────────────

// Front hit — reverse then spin 180°
void respondFront() {
  Serial.println("  >> FRONT: reverse + spin 180");
  setMotors(-PWM_REVERSE, PWM_REVERSE);  // reverse
  delay(300);
  setMotors(-PWM_TURN, -PWM_TURN);       // spin left 180°
  delay(400);
  setMotors(0, 0);
  delay(100);
}

// Rear hit — drive forward then spin
void respondRear() {
  Serial.println("  >> REAR: forward + spin");
  setMotors(PWM_REVERSE, -PWM_REVERSE);  // lunge forward
  delay(300);
  setMotors(-PWM_TURN, -PWM_TURN);       // spin left 180°
  delay(400);
  setMotors(0, 0);
  delay(100);
}

// Left side hit — turn hard right (away from left edge)
void respondLeft() {
  Serial.println("  >> LEFT SIDE: turn hard right");
  setMotors(PWM_TURN, PWM_TURN);         // spin right
  delay(350);
  setMotors(0, 0);
  delay(100);
}

// Right side hit — turn hard left (away from right edge)
void respondRight() {
  Serial.println("  >> RIGHT SIDE: turn hard left");
  setMotors(-PWM_TURN, -PWM_TURN);       // spin left
  delay(350);
  setMotors(0, 0);
  delay(100);
}

// ── Direction logic ───────────────────────────────────────────────────────────
void handleEdge(uint8_t edges) {
  bool fl = edges & (1 << 0);
  bool fr = edges & (1 << 1);
  bool rl = edges & (1 << 2);
  bool rr = edges & (1 << 3);

  bool front = fl || fr;
  bool rear  = rl || rr;
  bool left  = fl || rl;
  bool right = fr || rr;

  // Full side hits take priority over corner hits
  if (left && !right) {
    respondLeft();
  } else if (right && !left) {
    respondRight();
  } else if (front && !rear) {
    respondFront();
  } else if (rear && !front) {
    respondRear();
  } else {
    // Both sides or all sensors — just reverse and spin as safe default
    respondFront();
  }
}

// ── Status printing ───────────────────────────────────────────────────────────
void printStatus(uint8_t edges) {
  bool fl = edges & (1 << 0);
  bool fr = edges & (1 << 1);
  bool rl = edges & (1 << 2);
  bool rr = edges & (1 << 3);

  Serial.print("FL:"); Serial.print(fl ? "1" : "0");
  Serial.print("  FR:"); Serial.print(fr ? "1" : "0");
  Serial.print("  RL:"); Serial.print(rl ? "1" : "0");
  Serial.print("  RR:"); Serial.print(rr ? "1" : "0");

  if (edges) {
    Serial.print("  *** EDGE: ");
    if (fl) Serial.print("FL ");
    if (fr) Serial.print("FR ");
    if (rl) Serial.print("RL ");
    if (rr) Serial.print("RR ");
    Serial.print("***");
  } else {
    Serial.print("  [ CLEAR ]");
  }
  Serial.println();
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_EDGE_FL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_FR, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RR, INPUT_PULLDOWN);

  pinMode(PIN_MOTOR_L, OUTPUT);
  pinMode(PIN_MOTOR_R, OUTPUT);
  motorL.attach(PIN_MOTOR_L, 1000, 2000);
  motorR.attach(PIN_MOTOR_R, 1000, 2000);
  motorL.writeMicroseconds(1500);
  motorR.writeMicroseconds(1500);
  delay(3000);

  Serial.println("========================================");
  Serial.println("       EDGE DETECTION — DIRECTIONAL");
  Serial.println("========================================");
  Serial.println("FRONT hit -> reverse + spin 180");
  Serial.println("REAR  hit -> forward + spin");
  Serial.println("LEFT  hit -> turn hard right");
  Serial.println("RIGHT hit -> turn hard left");
  Serial.println("----------------------------------------");
  Serial.print("Sensor check: FL="); Serial.print(digitalRead(PIN_EDGE_FL));
  Serial.print(" FR="); Serial.print(digitalRead(PIN_EDGE_FR));
  Serial.print(" RL="); Serial.print(digitalRead(PIN_EDGE_RL));
  Serial.print(" RR="); Serial.println(digitalRead(PIN_EDGE_RR));
  Serial.println("========================================");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
uint32_t lastPrint = 0;
uint8_t  lastEdges = 0xFF;
bool     wasEdge   = false;

void loop() {
  uint8_t edges  = readEdgesDebounced();
  bool    isEdge = (edges != 0);

  if (millis() - lastPrint >= 150 || edges != lastEdges) {
    lastPrint = millis();
    printStatus(edges);
    lastEdges = edges;
  }

  if (isEdge) {
    handleEdge(edges);  // blocks for full manoeuvre, no stutter
  } else {
    setMotors(PWM_CREEP, -PWM_CREEP);
  }

  wasEdge = isEdge;
}