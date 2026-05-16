#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
#define MUX_ADDR     0x70
#define MUX_CH_TOF   0      // channel 0 on TCA9548A → front VL53L0X

// Distance bands matching production strategy thresholds
#define TOF_CONTACT_MM   120
#define TOF_ATTACK_MM    350
#define TOF_SCAN_MM      900

Adafruit_VL53L0X tof;

// ── Mux helpers ───────────────────────────────────────────────────────────────
void muxSelect(uint8_t ch) {
  Wire2.beginTransmission(MUX_ADDR);
  Wire2.write(1 << ch);
  Wire2.endTransmission();
  delayMicroseconds(50);
}

void muxDisableAll() {
  Wire2.beginTransmission(MUX_ADDR);
  Wire2.write(0);
  Wire2.endTransmission();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("========================================");
  Serial.println("          VL53L0X TOF TESTER");
  Serial.println("========================================");
  Serial.println("Bus  : Wire2 (SCL2=pin24, SDA2=pin25)");
  Serial.println("Mux  : TCA9548A @ 0x70, channel 0");
  Serial.println("Sensor: VL53L0X");
  Serial.println("----------------------------------------");

  Wire2.begin();
  Wire2.setClock(400000);

  muxDisableAll();
  muxSelect(MUX_CH_TOF);

  if (!tof.begin(0x29, false, &Wire2)) {
    Serial.println("ERROR: VL53L0X not found on mux channel 0.");
    Serial.println("Check wiring: SDA2->pin25, SCL2->pin24, mux power, sensor power.");
    while (1);
  }

  tof.setMeasurementTimingBudgetMicroSeconds(20000);  // 20ms — fast mode
  tof.startRangeContinuous();

  Serial.println("VL53L0X OK — continuous ranging started.");
  Serial.println("----------------------------------------");
  Serial.println("Bands:");
  Serial.println("  < 120mm  CONTACT (ram zone)");
  Serial.println("  120–350mm  ATTACK RANGE");
  Serial.println("  350–900mm  SCAN RANGE");
  Serial.println("  > 900mm  CLEAR");
  Serial.println("========================================");
  Serial.println();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  muxSelect(MUX_CH_TOF);

  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);

  // Range status codes:
  // 0 = valid, 4 = signal fail / out of range, others = hardware faults
  if (measure.RangeStatus == 4) {
    Serial.println("[ OUT OF RANGE ] No target detected (>2000mm or signal lost)");
  } else if (measure.RangeStatus != 0) {
    Serial.print("[ SENSOR FAULT ] RangeStatus=");
    Serial.print(measure.RangeStatus);
    Serial.println(" — check sensor alignment or power");
  } else {
    uint16_t mm = constrain(measure.RangeMilliMeter, 0, 2000);

    // Raw value
    Serial.print("Distance: ");
    Serial.print(mm);
    Serial.print(" mm  |  ");

    // Qualitative band
    if (mm < TOF_CONTACT_MM) {
      Serial.println("*** CONTACT — ram zone ***");
    } else if (mm < TOF_ATTACK_MM) {
      Serial.print("ATTACK RANGE  (");
      Serial.print(TOF_ATTACK_MM - mm);
      Serial.println("mm to contact threshold)");
    } else if (mm < TOF_SCAN_MM) {
      Serial.print("SCAN RANGE  (");
      Serial.print(TOF_SCAN_MM - mm);
      Serial.println("mm to scan threshold)");
    } else {
      Serial.println("CLEAR — beyond scan range");
    }
  }

  delay(100);  // 10Hz print rate — readable on serial monitor
}
