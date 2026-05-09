
#include <wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/*
Note that a lot of these are temp values that are to be updated later since we dont currently have the robot,
THIS IS NOT READY FOR TESTING YET
*/

//I2C addresses (to be confirmed later)
#define TCA_ADDR   0x70
#define MPU_ADDR   0x68

//pins 
#define Ratchet_pin 4 
//IR sensor pins (B = back , F = Front)
#define IR_FL 5 
#define IR_FR 6
#define IR_BL 7
#define IR_BR 8 

//Start module
#define STRT_MOD  9

//motors (left/right to be defined later)
#define Motor_1 10
#define Motor_2 11

//SDA/SCL (to be clarified later)
#define I2C_SDA 44
#define I2C_SCL 45

//Constants (to be perfected through testing)
#define ENGAGE_DIST_CM  30
#define BOUNDARY_VAL    600
#define ORBIT_1         180
#define ORBIT_2         255
#define FULL            255
#define REV_TIME_MS     250
#define STARTUP_DELAY   5000

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
