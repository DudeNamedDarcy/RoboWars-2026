#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L0X.h>

// Multiplexer I2C Address
#define TCAADDR 0x70

// Create sensor objects
Adafruit_MPU6050 mpu;
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

unsigned long lastUpdate = 0;
const int interval = 200;

// Helper function to switch TCA9548A ports
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  Wire.begin();

  // Initialize MPU6050 on Port 5
  tcaselect(5);
  if (!mpu.begin()) {
    Serial.println("Could not find MPU6050 on Port 5");
  }

  // Initialize VL53L0X #1 on Port 3
  tcaselect(3);
  if (!lox1.begin()) {
    Serial.println("Could not find VL53L0X on Port 3");
  }

  // Initialize VL53L0X #2 on Port 4
  tcaselect(4);
  if (!lox2.begin()) {
    Serial.println("Could not find VL53L0X on Port 4");
  }

  Serial.println("All sensors initialized!");
}

void loop() {
  if (millis() - lastUpdate >= interval) {
    lastUpdate = millis();

    // 1. Read MPU6050 (Port 5)
    tcaselect(5);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 2. Read ToF Sensor 1 (Port 3)
    tcaselect(3);
    VL53L0X_RangingMeasurementData_t measure1;
    lox1.rangingTest(&measure1, false);

    // 3. Read ToF Sensor 2 (Port 4)
    tcaselect(4);
    VL53L0X_RangingMeasurementData_t measure2;
    lox2.rangingTest(&measure2, false);

    // Print Data
    Serial.print("Dist1: "); Serial.print(measure1.RangeMilliMeter);
    Serial.print("mm | Dist2: "); Serial.print(measure2.RangeMilliMeter);
    Serial.print("mm | AccelX: "); Serial.println(a.acceleration.x);
  }
}