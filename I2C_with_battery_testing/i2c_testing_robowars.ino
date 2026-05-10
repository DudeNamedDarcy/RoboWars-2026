#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

#define TCA_ADDR 0x70
#define I2C_SDA 18
#define I2C_SCL 19

Adafruit_VL53L0X tof;

// MUX Communication, controls 8 different i2c buses: channels 0-7
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

uint16_t readDistance() {
  tcaSelect(0);
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);
  
  // Print the status so we can see what's actually happening
  if (measure.RangeStatus != 0) {
    Serial.print("Status Error: "); Serial.println(measure.RangeStatus);
    return 0;
  }
  return measure.RangeMilliMeter / 10;
}


void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open
  Wire.begin(); // Join I2C bus as Master
  Wire.setClock(400000); // Set I2C speed to 400kHz
  

  tcaSelect(0);
  if (!tof.begin()) {
    Serial.println("Failed to boot VL53L0X :( Check wiring/pullups!");
    while (1);
  }
  Serial.println("VL53L0X Ready!");
}

void loop() {
  Serial.println("Tick");
  uint16_t dist = readDistance();

  if (dist > 0) {
    Serial.print("Target "); Serial.print(dist); Serial.println("cm — :3");
    }

  delay(100);
}

