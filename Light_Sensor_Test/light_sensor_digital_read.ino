//testing light sensor (HW-870)
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L0X.h>

int lightSensorPin = 1;
float lightValue = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  lightValue = digitalRead(lightSensorPin);
  Serial.println(lightValue);

  if (lightValue >= 1){
    Serial.println("Hey it's 1!");
  }

  delay(1000);


}
