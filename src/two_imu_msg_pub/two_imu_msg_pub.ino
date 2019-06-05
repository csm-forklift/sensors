/**
 * To be used with an Arduino Mega or other microcontroller with
 * sufficient memory to store the required ROS variables. This code
 * reads the data from an Adafruit BNO055 IMU breakoutboard and 
 * publishes the data as a ROS sensor_msgs::Imu message type.
 * 
 * Adafruits Connection Diagram
 * https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 * 
 * Board Connection
 * MEGA      BNO055
 * 5V        Vin
 * GND       GND
 * D21       SCL
 * D22       SDA
 * 
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
sensor_msgs::Imu imu_data;
ros::Publisher imu_pub("arduino/imu", &imu_data);
sensors::ImuCalibStatus imu_calib_status;
ros::Publisher imu_status_pub("arduino/imu_status", &imu_calib_status);
sensor_msgs::Imu imu2_data;
ros::Publisher imu2_pub("arduino/imu2", &imu2_data);
sensors::ImuCalibStatus imu2_calib_status;
ros::Publisher imu2_status_pub("arduino/imu2_status", &imu2_calib_status);

// IMU Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, BNO055_ADDRESS_B);
imu::Quaternion quat;
imu::Vector<3> ang;
imu::Vector<3> lin;
imu::Vector<3> mag;
imu::Quaternion quat2;
imu::Vector<3> ang2;
imu::Vector<3> lin2;
imu::Vector<3> mag2;
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
  nh.advertise(imu2_pub);
  nh.advertise(imu2_status_pub);
  Serial.begin(57600);
  delay(100);

  // Set up debuggin LED
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);

  // Wait for BNO055 to begin before continuing
  if (!bno.begin()) {
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
  if (!bno2.begin()) {
    // Blinks four times a second if no connection is made
    while(1) {
      digitalWrite(DEBUG_LED, HIGH);
      delay(125);
      digitalWrite(DEBUG_LED, LOW);
      delay(125);
    }
  }

  // Get Default Calibration Parameters
  adafruit_bno055_offsets_t calibration_data;
  bno.getSensorOffsets(calibration_data);

  // Grab IMU calibration parameters, if parameter is not set, use default value
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
  // TODO: Need to add second set of calibration parameters for second IMU
  bno2.setSensorOffsets(calibration_data);
}

void loop() {
  // Get IMU data from sensor
  quat = bno.getQuat();
  ang = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Set header
  imu_data.header.stamp = nh.now();
  imu_data.header.frame_id = "imu_link";

  // Set orientation
  imu_data.orientation.x = quat.x();
  imu_data.orientation.y = quat.y();
  imu_data.orientation.z = quat.z();
  imu_data.orientation.w = quat.w();

  // Set angular velocity
  imu_data.angular_velocity.x = ang.x();
  imu_data.angular_velocity.y = ang.y();
  imu_data.angular_velocity.z = ang.z();

  // Set linear acceleration
  imu_data.linear_acceleration.x = lin.x();
  imu_data.linear_acceleration.y = lin.y();
  imu_data.linear_acceleration.z = lin.z();

  // Grab calibration status
  bno.getCalibration(&imu_calib_status.system, &imu_calib_status.gyro, 
                     &imu_calib_status.accel, &imu_calib_status.mag);

  // Get Second IMU data from sensor
  quat2 = bno2.getQuat();
  ang2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin2 = bno2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  mag2 = bno2.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Set header
  imu2_data.header.stamp = nh.now();
  imu2_data.header.frame_id = "imu2_link";

  // Set orientation
  imu2_data.orientation.x = quat2.x();
  imu2_data.orientation.y = quat2.y();
  imu2_data.orientation.z = quat2.z();
  imu2_data.orientation.w = quat2.w();

  // Set angular velocity
  imu2_data.angular_velocity.x = ang2.x();
  imu2_data.angular_velocity.y = ang2.y();
  imu2_data.angular_velocity.z = ang2.z();

  // Set linear acceleration
  imu2_data.linear_acceleration.x = lin2.x();
  imu2_data.linear_acceleration.y = lin2.y();
  imu2_data.linear_acceleration.z = lin2.z();

  // Grab calibration status
  bno2.getCalibration(&imu2_calib_status.system, &imu2_calib_status.gyro, 
                     &imu2_calib_status.accel, &imu2_calib_status.mag);

  // Publish messages
  imu_pub.publish(&imu_data);
  imu_status_pub.publish(&imu_calib_status);
  imu2_pub.publish(&imu2_data);
  imu2_status_pub.publish(&imu2_calib_status);

  nh.spinOnce();
  delay(1);
}
