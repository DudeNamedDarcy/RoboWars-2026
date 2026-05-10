#include <Servo.h>

#define MOTOR_1_PIN 10

Servo motor1;

void setup() {
  Serial.begin(115200);
  motor1.attach(MOTOR_1_PIN);

  // Arming sequence
  Serial.println("Arming ESC...");
  motor1.writeMicroseconds(1500);
  delay(3000);
  Serial.println("Armed");
}

void loop() {
  // Forward half speed
  Serial.println("Forward - half");
  motor1.writeMicroseconds(1750);
  delay(2000);

  // Stop
  Serial.println("Stop");
  motor1.writeMicroseconds(1500);
  delay(2000);

  // Forward full
  Serial.println("Forward - full");
  motor1.writeMicroseconds(2000);
  delay(2000);

  // Stop
  Serial.println("Stop");
  motor1.writeMicroseconds(1500);
  delay(2000);

  // Reverse half speed
  Serial.println("Reverse - half");
  motor1.writeMicroseconds(1250);
  delay(2000);

  // Stop
  Serial.println("Stop");
  motor1.writeMicroseconds(1500);
  delay(2000);

  // Reverse full
  Serial.println("Reverse - full");
  motor1.writeMicroseconds(1000);
  delay(2000);

  // Stop
  Serial.println("Stop");
  motor1.writeMicroseconds(1500);
  delay(2000);
}