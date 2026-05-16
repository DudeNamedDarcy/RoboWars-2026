#include <Servo.h>

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PIN_EDGE_FL  5
#define PIN_EDGE_FR  6
#define PIN_EDGE_RL  7
#define PIN_EDGE_RR  8
#define PIN_MOTOR_L 10
#define PIN_MOTOR_R 11

// ── Motor ─────────────────────────────────────────────────────────────────────
#define PWM_MAX    255
#define PWM_CREEP   70   // slow forward crawl — enough to feel the stop without flying off

Servo motorL, motorR;

void setMotors(int l, int r) {
  l = constrain(l, -PWM_MAX, PWM_MAX);
  r = constrain(r, -PWM_MAX, PWM_MAX);
  motorL.writeMicroseconds(map(l, -PWM_MAX, PWM_MAX, 1000, 2000));
  motorR.writeMicroseconds(map(r, -PWM_MAX, PWM_MAX, 1000, 2000));
}

// ── Edge reading ──────────────────────────────────────────────────────────────

// Returns a 4-bit mask of currently HIGH sensors.
// Bit 0 = FL, Bit 1 = FR, Bit 2 = RL, Bit 3 = RR
uint8_t sampleEdges() {
  uint8_t mask = 0;
  if (digitalRead(PIN_EDGE_FL)) mask |= (1 << 0);
  if (digitalRead(PIN_EDGE_FR)) mask |= (1 << 1);
  if (digitalRead(PIN_EDGE_RL)) mask |= (1 << 2);
  if (digitalRead(PIN_EDGE_RR)) mask |= (1 << 3);
  return mask;
}

// Debounced read: takes two samples 500µs apart and ANDs them together.
// A sensor bit is only set if it was HIGH on both reads, rejecting noise spikes.
uint8_t readEdgesDebounced() {
  uint8_t first = sampleEdges();
  if (first == 0) return 0;
  delayMicroseconds(500);
  return sampleEdges() & first;
}

void printStatus(uint8_t edges) {
  bool fl = edges & (1 << 0);
  bool fr = edges & (1 << 1);
  bool rl = edges & (1 << 2);
  bool rr = edges & (1 << 3);

  Serial.print("FL:");  Serial.print(fl ? "1" : "0");
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

// Prints which logical groups are active (front / rear / left / right).
// Runs once per edge event to give extra context.
void printGroups(uint8_t edges) {
  bool fl = edges & (1 << 0);
  bool fr = edges & (1 << 1);
  bool rl = edges & (1 << 2);
  bool rr = edges & (1 << 3);

  Serial.print("  Groups triggered:");
  if (fl || fr) Serial.print(" FRONT");
  if (rl || rr) Serial.print(" REAR");
  if (fl || rl) Serial.print(" LEFT");
  if (fr || rr) Serial.print(" RIGHT");
  Serial.println();
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // Edge sensors — INPUT_PULLDOWN so a disconnected sensor reads LOW (safe default)
  pinMode(PIN_EDGE_FL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_FR, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RL, INPUT_PULLDOWN);
  pinMode(PIN_EDGE_RR, INPUT_PULLDOWN);

  // Motors
  pinMode(PIN_MOTOR_L, OUTPUT);
  pinMode(PIN_MOTOR_R, OUTPUT);
  motorL.attach(PIN_MOTOR_L, 1000, 2000);
  motorR.attach(PIN_MOTOR_R, 1000, 2000);
  setMotors(0, 0);   // 1500µs neutral — arms ESC
  delay(2000);

  Serial.println("========================================");
  Serial.println("       EDGE DETECTION TEST");
  Serial.println("========================================");
  Serial.println("Robot creeps forward at PWM_CREEP (70).");
  Serial.println("Any edge hit -> motors stop instantly.");
  Serial.println("Clear all sensors -> motors resume.");
  Serial.println("----------------------------------------");
  Serial.println("Sensor check on dark surface (all = 0):");
  Serial.print("  FL="); Serial.print(digitalRead(PIN_EDGE_FL));
  Serial.print("  FR="); Serial.print(digitalRead(PIN_EDGE_FR));
  Serial.print("  RL="); Serial.print(digitalRead(PIN_EDGE_RL));
  Serial.print("  RR="); Serial.println(digitalRead(PIN_EDGE_RR));
  Serial.println("----------------------------------------");
  Serial.println("Format: FL  FR  RL  RR  status");
  Serial.println("========================================");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
uint32_t lastPrint    = 0;
uint8_t  lastEdges    = 0xFF;   // force a print on first loop
bool     wasEdge      = false;

void loop() {
  uint8_t edges = readEdgesDebounced();
  bool    isEdge = (edges != 0);

  // Print status every 150ms, or immediately when state changes
  if (millis() - lastPrint >= 150 || edges != lastEdges) {
    lastPrint = millis();
    printStatus(edges);

    // Print group summary once on the rising edge of a detection event
    if (isEdge && !wasEdge) {
      printGroups(edges);
    }

    lastEdges = edges;
  }

  // Motor behaviour: creep forward when clear, stop on any edge
  if (isEdge) {
    setMotors(0, 0);
  } else {
    setMotors(PWM_CREEP, PWM_CREEP);
  }

  wasEdge = isEdge;
}
