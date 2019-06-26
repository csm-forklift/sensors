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


// ROS Includes
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensors/ImuCalibStatus.h>

// IMU Includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ROS Objects
ros::NodeHandle nh;
sensor_msgs::Imu imu0_data;
ros::Publisher imu0_pub("arduino/imu0", &imu0_data);
sensors::ImuCalibStatus imu0_calib_status;
ros::Publisher imu0_status_pub("arduino/imu0_status", &imu0_calib_status);
sensor_msgs::Imu imu1_data;
ros::Publisher imu1_pub("arduino/imu1", &imu1_data);
sensors::ImuCalibStatus imu1_calib_status;
ros::Publisher imu1_status_pub("arduino/imu1_status", &imu1_calib_status);

// IMU Objects
Adafruit_BNO055 bno0 = Adafruit_BNO055(55);
Adafruit_BNO055 bno1 = Adafruit_BNO055(56, BNO055_ADDRESS_B);

// Configuration Variables
byte PAGE1 = 0x01;
byte PAGE0 = 0x00;
byte BNO055_ACC_CONFIG_ADDR = 0x08;
byte BNO055_GYRO_CONFIG0_ADDR = 0x0A;

// Data Variables
imu::Quaternion quat0;
imu::Vector<3> ang0;
imu::Vector<3> lin0;
imu::Vector<3> mag0;
imu::Quaternion quat1;
imu::Vector<3> ang1;
imu::Vector<3> lin1;
imu::Vector<3> mag1;
// comment out the three vectors above and use this variable, 'vec',
// for each of the three vector values you read if you are low on memory.
//imu::Vector<3> vec;

// Use LED to signal debugging messages
const int DEBUG_LED = 13;

void setup() {
  // Start ROS Node
  nh.initNode();
  nh.advertise(imu0_pub);
  nh.advertise(imu0_status_pub);
  nh.advertise(imu1_pub);
  nh.advertise(imu1_status_pub);
  Serial.begin(57600);
  delay(100);

  // Set up debuggin LED
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);

  // Wait for BNO055 to begin before continuing
  if (!bno0.begin(Adafruit_BNO055::OPERATION_MODE_AMG)) {
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

  // Wait for second BNO055 to begin before continuing
  if (!bno1.begin(Adafruit_BNO055::OPERATION_MODE_AMG)) {
    // Blinks four times a second if no connection is made
    while(1) {
      digitalWrite(DEBUG_LED, HIGH);
      delay(125);
      digitalWrite(DEBUG_LED, LOW);
      delay(125);
    }
  }

  //===== Configure the accelerometer range to +/-2g and gyroscope to +/-250dps
  // IMU0, accelerometer
  bno0.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  bno0.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, PAGE1); // set to page 1
  byte acc_current0 = bno0.read8(BNO055_ACC_CONFIG_ADDR);
  byte acc_2g_range_mask0 = (1 << 1) | (1 << 0);
  byte acc_config_bits0 = (~acc_2g_range_mask0 & acc_current0);
  bno0.write8(BNO055_ACC_CONFIG_ADDR, acc_config_bits0);
  // IMU0, gyroscope
  byte gyro_current0 = bno0.read8(BNO055_GYRO_CONFIG0_ADDR);
  byte gyro_250dps_range_mask0 = (1 << 1) | (1 << 0);
  byte gyro_config_bits0 = (gyro_250dps_range_mask0 | gyro_current0);
  bno0.write8(BNO055_GYRO_CONFIG0_ADDR, gyro_config_bits0);
  // Set back to page 0
  bno0.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, PAGE0);
  bno0.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);
  // IMU1, accelerometer
  bno1.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  bno1.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, PAGE1); // set to page 1
  byte acc_current1 = bno1.read8(BNO055_ACC_CONFIG_ADDR);
  byte acc_2g_range_mask1 = (1 << 1) | (1 << 0);
  byte acc_config_bits1 = (~acc_2g_range_mask1 & acc_current1);
  bno1.write8(BNO055_ACC_CONFIG_ADDR, acc_config_bits1);
  // IMU1, gyroscope
  byte gyro_current1 = bno1.read8(BNO055_GYRO_CONFIG0_ADDR);
  byte gyro_250dps_range_mask1 = (1 << 1) | (1 << 0);
  byte gyro_config_bits1 = (gyro_250dps_range_mask1 | gyro_current1);
  bno1.write8(BNO055_GYRO_CONFIG0_ADDR, gyro_config_bits1);
  // Set back to page 0
  bno1.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, PAGE0);
  bno1.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);

  //===== Get Default Calibration Parameters
  adafruit_bno055_offsets_t calibration_data;
  bno0.getSensorOffsets(calibration_data);

  // Grab IMU0 calibration parameters, if parameter is not set, use default value
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
  int16_t mag_offset_x;
  int16_t mag_offset_y;
  int16_t mag_offset_z;
  int16_t accel_radius;
  int16_t mag_radius;

  // Needs a delay before attempting to load parameters
  delay(1000);
  
  if (!nh.getParam("imu0/calibration/accel_offset_x", &accel_offset_x)) {
    accel_offset_x = calibration_data.accel_offset_x;
  }
  if (!nh.getParam("imu0/calibration/accel_offset_y", &accel_offset_y)) {
    accel_offset_y = calibration_data.accel_offset_y;
  }
  if (!nh.getParam("imu0/calibration/accel_offset_z", &accel_offset_z)) {
    accel_offset_z = calibration_data.accel_offset_z;
  }
  if (!nh.getParam("imu0/calibration/accel_radius", &accel_radius)) {
    accel_radius = calibration_data.accel_radius;
  }
  if (!nh.getParam("imu0/calibration/gyro_offset_x", &gyro_offset_x)) {
    gyro_offset_x = calibration_data.gyro_offset_x;
  }
  if (!nh.getParam("imu0/calibration/gyro_offset_y", &gyro_offset_y)) {
    gyro_offset_y = calibration_data.gyro_offset_y;
  }
  if (!nh.getParam("imu0/calibration/gyro_offset_z", &gyro_offset_z)) {
    gyro_offset_z = calibration_data.gyro_offset_z;
  }
  if (!nh.getParam("imu0/calibration/mag_offset_x", &mag_offset_x)) {
    mag_offset_x = calibration_data.mag_offset_x;
  }
  if (!nh.getParam("imu0/calibration/mag_offset_y", &mag_offset_y)) {
    mag_offset_y = calibration_data.mag_offset_y;
  }
  if (!nh.getParam("imu0/calibration/mag_offset_z", &mag_offset_z)) {
    mag_offset_z = calibration_data.mag_offset_z;
  }
  if (!nh.getParam("imu0/calibration/mag_radius", &mag_radius)) {
    mag_radius = calibration_data.mag_radius;
  }

  // Upload parameters to IMU
  calibration_data.accel_offset_x = accel_offset_x;
  calibration_data.accel_offset_y = accel_offset_y;
  calibration_data.accel_offset_z = accel_offset_z;
  calibration_data.accel_radius = accel_radius;
  calibration_data.gyro_offset_x = gyro_offset_x;
  calibration_data.gyro_offset_y = gyro_offset_y;
  calibration_data.gyro_offset_z = gyro_offset_z;
  calibration_data.mag_offset_x = mag_offset_x;
  calibration_data.mag_offset_y = mag_offset_y;
  calibration_data.mag_offset_z = mag_offset_z;
  calibration_data.mag_radius = mag_radius;
  bno0.setSensorOffsets(calibration_data);
  
  // Load Calibration Parameters for IMU 1
  bno1.getSensorOffsets(calibration_data); // grab defaults first
  
  if (!nh.getParam("imu1/calibration/accel_offset_x", &accel_offset_x)) {
    accel_offset_x = calibration_data.accel_offset_x;
  }
  if (!nh.getParam("imu1/calibration/accel_offset_y", &accel_offset_y)) {
    accel_offset_y = calibration_data.accel_offset_y;
  }
  if (!nh.getParam("imu1/calibration/accel_offset_z", &accel_offset_z)) {
    accel_offset_z = calibration_data.accel_offset_z;
  }
  if (!nh.getParam("imu1/calibration/accel_radius", &accel_radius)) {
    accel_radius = calibration_data.accel_radius;
  }
  if (!nh.getParam("imu1/calibration/gyro_offset_x", &gyro_offset_x)) {
    gyro_offset_x = calibration_data.gyro_offset_x;
  }
  if (!nh.getParam("imu1/calibration/gyro_offset_y", &gyro_offset_y)) {
    gyro_offset_y = calibration_data.gyro_offset_y;
  }
  if (!nh.getParam("imu1/calibration/gyro_offset_z", &gyro_offset_z)) {
    gyro_offset_z = calibration_data.gyro_offset_z;
  }
  if (!nh.getParam("imu1/calibration/mag_offset_x", &mag_offset_x)) {
    mag_offset_x = calibration_data.mag_offset_x;
  }
  if (!nh.getParam("imu1/calibration/mag_offset_y", &mag_offset_y)) {
    mag_offset_y = calibration_data.mag_offset_y;
  }
  if (!nh.getParam("imu1/calibration/mag_offset_z", &mag_offset_z)) {
    mag_offset_z = calibration_data.mag_offset_z;
  }
  if (!nh.getParam("imu1/calibration/mag_radius", &mag_radius)) {
    mag_radius = calibration_data.mag_radius;
  }

  // Upload parameters to IMU 1
  calibration_data.accel_offset_x = accel_offset_x;
  calibration_data.accel_offset_y = accel_offset_y;
  calibration_data.accel_offset_z = accel_offset_z;
  calibration_data.accel_radius = accel_radius;
  calibration_data.gyro_offset_x = gyro_offset_x;
  calibration_data.gyro_offset_y = gyro_offset_y;
  calibration_data.gyro_offset_z = gyro_offset_z;
  calibration_data.mag_offset_x = mag_offset_x;
  calibration_data.mag_offset_y = mag_offset_y;
  calibration_data.mag_offset_z = mag_offset_z;
  calibration_data.mag_radius = mag_radius;
  bno1.setSensorOffsets(calibration_data);
}

void loop() {
  // Get IMU0 data from sensor
  ang0 = bno0.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin0 = bno0.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  mag0 = bno0.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Set header
  imu0_data.header.stamp = nh.now();
  imu0_data.header.frame_id = "imu0_link";

  // Set orientation
  imu0_data.orientation.x = 0;
  imu0_data.orientation.y = 0;
  imu0_data.orientation.z = 0;
  imu0_data.orientation.w = 1;

  // Set angular velocity
  imu0_data.angular_velocity.x = ang0.x();
  imu0_data.angular_velocity.y = ang0.y();
  imu0_data.angular_velocity.z = ang0.z();

  // Set linear acceleration
  imu0_data.linear_acceleration.x = lin0.x();
  imu0_data.linear_acceleration.y = lin0.y();
  imu0_data.linear_acceleration.z = lin0.z();

  // Grab calibration status
  bno0.getCalibration(&imu0_calib_status.system, &imu0_calib_status.gyro, 
                     &imu0_calib_status.accel, &imu0_calib_status.mag);

  // Get IMU1 data from sensor
  ang1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin1 = bno1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  mag1 = bno1.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Set header
  imu1_data.header.stamp = nh.now();
  imu1_data.header.frame_id = "imu1_link";

  // Set orientation
  imu1_data.orientation.x = 0;
  imu1_data.orientation.y = 0;
  imu1_data.orientation.z = 0;
  imu1_data.orientation.w = 1;

  // Set angular velocity
  imu1_data.angular_velocity.x = ang1.x();
  imu1_data.angular_velocity.y = ang1.y();
  imu1_data.angular_velocity.z = ang1.z();

  // Set linear acceleration
  imu1_data.linear_acceleration.x = lin1.x();
  imu1_data.linear_acceleration.y = lin1.y();
  imu1_data.linear_acceleration.z = lin1.z();

  // Grab calibration status
  bno1.getCalibration(&imu1_calib_status.system, &imu1_calib_status.gyro, 
                     &imu1_calib_status.accel, &imu1_calib_status.mag);

  // Publish messages
  imu0_pub.publish(&imu0_data);
  imu0_status_pub.publish(&imu0_calib_status);
  imu1_pub.publish(&imu1_data);
  imu1_status_pub.publish(&imu1_calib_status);

  nh.spinOnce();
  delay(1);
}
