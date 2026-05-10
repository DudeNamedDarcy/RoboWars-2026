#define START_MODULE_PIN 5

void setup() {
  Serial.begin(115200);
  while(!Serial); 

  // CHANGE: Use INPUT instead of INPUT_PULLUP
  pinMode(START_MODULE_PIN, INPUT); 

  Serial.println("MicroStart Monitor Active");
}

void loop() {
  int status = digitalRead(START_MODULE_PIN);
  
  if (status == HIGH) {
    Serial.println(">>> RUNNING (Logic 1) <<<");
  } else {
    Serial.println("STOPPED (Logic 0)");
  }
  delay(200);
}