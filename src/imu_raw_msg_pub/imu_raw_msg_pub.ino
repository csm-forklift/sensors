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

// Constants
#define M_PI 3.14159265358979323846  /* pi */

// ROS Objects
ros::NodeHandle nh;
sensor_msgs::Imu imu_data;
ros::Publisher imu_pub("arduino/imu", &imu_data);
sensors::ImuCalibStatus imu_calib_status;
ros::Publisher imu_status_pub("arduino/imu_status", &imu_calib_status);

// IMU Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Configuration Variables
byte PAGE1 = 0x01;
byte PAGE0 = 0x00;
byte BNO055_ACC_CONFIG_ADDR = 0x08;
byte BNO055_GYRO_CONFIG0_ADDR = 0x0A;

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
  // Start ROS Node
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(imu_status_pub);
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
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, PAGE1); // set to page 1
  byte acc_current = bno.read8(BNO055_ACC_CONFIG_ADDR);
  byte acc_2g_range_mask = (1 << 1) | (1 << 0);
  byte acc_config_bits = (~acc_2g_range_mask & acc_current);
  bno.write8(BNO055_ACC_CONFIG_ADDR, acc_config_bits);
  // imu, gyroscope
  byte gyro_current = bno.read8(BNO055_GYRO_CONFIG0_ADDR);
  byte gyro_250dps_range_mask = (1 << 1) | (1 << 0);
  byte gyro_config_bits = (gyro_250dps_range_mask | gyro_current);
  bno.write8(BNO055_GYRO_CONFIG0_ADDR, gyro_config_bits);
  // Set back to page 0
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, PAGE0);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG);

  //===== Get Default Calibration Parameters
  adafruit_bno055_offsets_t calibration_data;
  bno.getSensorOffsets(calibration_data);

  // Grab imu calibration parameters, if parameter is not set, use default value
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
  
  if (!nh.getParam("imu/calibration/accel_offset_x", &accel_offset_x)) {
    accel_offset_x = calibration_data.accel_offset_x;
  }
  if (!nh.getParam("imu/calibration/accel_offset_y", &accel_offset_y)) {
    accel_offset_y = calibration_data.accel_offset_y;
  }
  if (!nh.getParam("imu/calibration/accel_offset_z", &accel_offset_z)) {
    accel_offset_z = calibration_data.accel_offset_z;
  }
  if (!nh.getParam("imu/calibration/accel_radius", &accel_radius)) {
    accel_radius = calibration_data.accel_radius;
  }
  if (!nh.getParam("imu/calibration/gyro_offset_x", &gyro_offset_x)) {
    gyro_offset_x = calibration_data.gyro_offset_x;
  }
  if (!nh.getParam("imu/calibration/gyro_offset_y", &gyro_offset_y)) {
    gyro_offset_y = calibration_data.gyro_offset_y;
  }
  if (!nh.getParam("imu/calibration/gyro_offset_z", &gyro_offset_z)) {
    gyro_offset_z = calibration_data.gyro_offset_z;
  }
  if (!nh.getParam("imu/calibration/mag_offset_x", &mag_offset_x)) {
    mag_offset_x = calibration_data.mag_offset_x;
  }
  if (!nh.getParam("imu/calibration/mag_offset_y", &mag_offset_y)) {
    mag_offset_y = calibration_data.mag_offset_y;
  }
  if (!nh.getParam("imu/calibration/mag_offset_z", &mag_offset_z)) {
    mag_offset_z = calibration_data.mag_offset_z;
  }
  if (!nh.getParam("imu/calibration/mag_radius", &mag_radius)) {
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
  bno.setSensorOffsets(calibration_data);
}

void loop() {
  // Get imu data from sensor
  ang = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Set header
  imu_data.header.stamp = nh.now();
  imu_data.header.frame_id = "imu_link";

  // Set orientation
  imu_data.orientation.x = 0;
  imu_data.orientation.y = 0;
  imu_data.orientation.z = 0;
  imu_data.orientation.w = 1;

  // Set angular velocity (original data in Degrees Per Second, need to convert
  // to Radians Per Second)
  imu_data.angular_velocity.x = ang.x()*(M_PI/180.0);
  imu_data.angular_velocity.y = ang.y()*(M_PI/180.0);
  imu_data.angular_velocity.z = ang.z()*(M_PI/180.0);

  // Set linear acceleration
  imu_data.linear_acceleration.x = lin.x();
  imu_data.linear_acceleration.y = lin.y();
  imu_data.linear_acceleration.z = lin.z();

  // Grab calibration status
  bno.getCalibration(&imu_calib_status.system, &imu_calib_status.gyro, 
                     &imu_calib_status.accel, &imu_calib_status.mag);

  // Publish messages
  imu_pub.publish(&imu_data);
  imu_status_pub.publish(&imu_calib_status);

  nh.spinOnce();
  delay(1);
}
