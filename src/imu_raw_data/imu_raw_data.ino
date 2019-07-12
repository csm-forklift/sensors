/**
 * To be used with an Arduino Mega or other microcontroller with
 * sufficient memory to store the required ROS variables. This code
 * reads the data from an Adafruit BNO055 IMU breakoutboard and 
 * publishes the data as a ROS sensor_msgs::Imu message type.
 * 
 * This version is intended to use the 
 * 
 * Adafruits Connection Diagram
 * https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 * 
 * Board Connection
 * MEGA      BNO055 (same pins for both because they share I2C lines)
 * 5V        Vin
 * GND       GND
 * D21       SCL
 * D22       SDA
 * 
 * IMU 0
 * ID on back: 4517
 * 
 * IMU 1
 * ID on back: 0519
 */
// IMU Includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Constants
#define M_PI 3.14159265358979323846  /* pi */

// IMU Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Configuration Variables
byte PAGE1 = 0x01;
byte PAGE0 = 0x00;
byte BNO055_ACC_CONFIG_ADDR = 0x08;
byte BNO055_GYRO_CONFIG0_ADDR = 0x0A;
byte BNO055_UNIT_SEL_ADDR = 0x3B;
// Data Variables
imu::Quaternion quat;
imu::Vector<3> ang;
imu::Vector<3> lin;
imu::Vector<3> mag;
// comment out the three vectors above and use this variable, 'vec',
// for each of the three vector values you read if you are low on memory.
//imu::Vector<3> vec;

// Use LED to signal debugging messages
const int DEBUG_LED = 13;

void setup() {

  Serial.begin(57600);
  delay(100);

  // Set up debuggin LED
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);

  // Wait for BNO055 to begin before continuing
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_AMG)) {
    //Serial.println("No BNO055 detected.");
    // Blinks twice a second if no connection is made
    while(1) {
      digitalWrite(DEBUG_LED, HIGH);
      delay(250);
      digitalWrite(DEBUG_LED, LOW);
      delay(250);
      digitalWrite(DEBUG_LED, HIGH);
      delay(250);
      digitalWrite(DEBUG_LED, LOW);
      delay(250);
    }
  }

  //===== Configure the accelerometer range to +/-2g and gyroscope to +/-250dps
  // imu, accelerometer
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  delay(80);
  
  byte current_unit_select = bno.read8(Adafruit_BNO055::BNO055_UNIT_SEL_ADDR);
  byte gyro_units_to_rps = B00000111;
  byte rps_unit_bytes = (current_unit_select | gyro_units_to_rps);
  Serial.println(rps_unit_bytes, BIN);
  Serial.println(bno.write8(Adafruit_BNO055::BNO055_UNIT_SEL_ADDR,rps_unit_bytes));
  delay(50);
  Serial.println(bno.read8(Adafruit_BNO055::BNO055_UNIT_SEL_ADDR), BIN);





  
  Serial.println(bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, PAGE1)); // set to page 1
  Serial.println(bno.read8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR));
  byte acc_current = bno.read8(BNO055_ACC_CONFIG_ADDR);
  byte acc_2g_range_mask = (1 << 1) | (1 << 0);
  byte acc_config_bits = (~acc_2g_range_mask & acc_current);
  Serial.println(bno.write8(BNO055_ACC_CONFIG_ADDR, acc_config_bits));
  delay(50);
  // imu, gyroscope
  byte gyro_current = bno.read8(BNO055_GYRO_CONFIG0_ADDR);
  //byte gyro_250dps_range_mask = (1 << 1) | (1 << 0);
  //byte gyro_config_bits = (gyro_250dps_range_mask | gyro_current);

  byte gyro_125dps_range_mask =  B00000100;
  byte gyro_125dps_range_mask2 = B11111100;
  byte gyro_config_bits = (gyro_125dps_range_mask | gyro_current) & gyro_125dps_range_mask2;

  bno.write8(BNO055_GYRO_CONFIG0_ADDR, gyro_config_bits);
  delay(50);
  // Set back to page 0
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR,PAGE0);


  delay(50);

  
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);
  delay(50);

  
  //===== Get Default Calibration Parameters
  adafruit_bno055_offsets_t calibration_data;
  bno.getSensorOffsets(calibration_data);
}

void loop() {
  // Get imu data from sensor
  ang = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  
  mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  /*
  Serial.print("Angular - ");
  Serial.print("x: ");
  Serial.print(ang.x());
  Serial.print(", y: ");
  Serial.print(ang.y());
  Serial.print(", z: ");
  Serial.print(ang.z());

  Serial.print(", Acceleration - ");
  Serial.print("x: ");
  Serial.print(lin.x());
  Serial.print(", y: ");
  Serial.print(lin.y());
  Serial.print(", z: ");
  Serial.println(lin.z());
 */

  delay(100);
}
