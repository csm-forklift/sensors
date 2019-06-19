#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

byte page = 0x01;
byte BNO055_ACC_CONFIG_ADDR = 0x08;
byte BNO055_GYRO_CONFIG0_ADDR = 0x0A;

void setup() {
    Serial.begin(57600);
    delay(1000);

    if (!bno.begin()) {
      Serial.println("No BNO055 detected.");
      while(1);
    }

    // Set to config mode
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);

    // Change to page 1
    bool page_changed = bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, page);
    Serial.println(bno.read8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR), BIN);

    // Change Accelerometer to 2g range
    byte acc_current = bno.read8(BNO055_ACC_CONFIG_ADDR);
    byte acc_range_bits = (1 << 1) | (1 << 0);
    Serial.print("Current: ");
    Serial.println(acc_current, BIN);
    Serial.print("Mask: ");
    Serial.println(acc_range_bits, BIN);
    byte acc_new = (~acc_range_bits & acc_current);
    Serial.print("New: ");
    Serial.println(acc_new, BIN);
    bno.write8(BNO055_ACC_CONFIG_ADDR, acc_new);
    
    // Change Gyroscope to 250dps range
    byte gyro_current = bno.read8(BNO055_GYRO_CONFIG0_ADDR);
    byte gyro_range_bits = (1 << 1) | (1 << 0);
    Serial.print("Current: ");
    Serial.println(gyro_current, BIN);
    Serial.print("Mask: ");
    Serial.println(gyro_range_bits, BIN);
    byte gyro_new = (gyro_range_bits | gyro_current);
    Serial.print("New: ");
    Serial.println(gyro_new, BIN);
    bno.write8(BNO055_GYRO_CONFIG0_ADDR, gyro_new);

    // Change mode back to IMU_PLUS
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
}

void loop() {
  // Read the Accelerometer config register
  byte acc_config = bno.read8(BNO055_ACC_CONFIG_ADDR);

  // Print
  Serial.print("ACC_CONFIG: ");
  Serial.println(acc_config, BIN);

  // Read the Gyroscope config register
  byte gyro_config = bno.read8(BNO055_GYRO_CONFIG0_ADDR);
  
  // Print
  Serial.print("GYRO_CONFIG0: ");
  Serial.println(gyro_config, BIN);

  // Delay
  delay(1000);

}
