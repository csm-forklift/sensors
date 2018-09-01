// ROS Includes
#include <ros.h>
#include <encoders/ImuArray.h>
#include <encoders/ImuMag.h>
#include <encoders/ImuCalibStatus.h>

// IMU Includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


ros::NodeHandle nh;

encoders::ImuArray imu_data;
ros::Publisher imu_pub("imu/data_array", &imu_data);
encoders::ImuMag imu_mag;
ros::Publisher mag_pub("imu/mag_array", &imu_mag);
encoders::ImuCalibStatus imu_calib_status;
ros::Publisher imu_status_pub("imu/status", &imu_calib_status);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

imu::Quaternion quat;
//imu::Vector<3> ang;
//imu::Vector<3> lin;
//imu::Vector<3> mag;
imu::Vector<3> vec;

void setup() {
  // Start ROS Node
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  nh.advertise(imu_status_pub);
  Serial.begin(57600);
  delay(100);

  // Wait for BNO055 to begin before continuing
  if (!bno.begin()) {
    //Serial.println("No BNO055 detected.");
    while(1);
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
}

void loop() {
//  // Get IMU data from sensor
//  imu::Quaternion quat = bno.getQuat(); 
//  imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat = bno.getQuat(); 
//  ang = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  lin = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//  mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //sensors_event_t event;
  //bno.getEvent(&event);

  /*float imuArray[] = { quat.w(), quat.x(), quat.y(), quat.z(),
                      ang.x(), ang.y(), ang.z(),
                      lin.x(), lin.y(), lin.z()};
  */

//  // Convert data to ROS IMU message
//  imu_data.orientation.w = quat.w();
//  imu_data.orientation.x = quat.x();
//  imu_data.orientation.y = quat.y();
//  imu_data.orientation.z = quat.z();
//  
//  imu_data.angular_velocity.x = ang.x();
//  imu_data.angular_velocity.y = ang.y();
//  imu_data.angular_velocity.z = ang.z();
//  
//  imu_data.linear_acceleration.x = lin.x();
//  imu_data.linear_acceleration.y = lin.y();
//  imu_data.linear_acceleration.z = lin.z();

//  imu_data.data[0] = quat.w();
//  imu_data.data[1] = quat.x();
//  imu_data.data[2] = quat.y();
//  imu_data.data[3] = quat.z();
//  imu_data.data[4] = ang.x();
//  imu_data.data[5] = ang.y();
//  imu_data.data[6] = ang.z();
//  imu_data.data[7] = lin.x();
//  imu_data.data[8] = lin.y();
//  imu_data.data[9] = lin.z();

  // Get IMU Data
  
  imu_data.data[0] = quat.w();
  imu_data.data[1] = quat.x();
  imu_data.data[2] = quat.y();
  imu_data.data[3] = quat.z();
  vec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_data.data[4] = vec.x();
  imu_data.data[5] = vec.y();
  imu_data.data[6] = vec.z();
  vec = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_data.data[7] = vec.x();
  imu_data.data[8] = vec.y();
  imu_data.data[9] = vec.z();

//  Serial.print("x: ");
//  Serial.print(mag.x());
//  Serial.print(", y: ");
//  Serial.print(mag.y());
//  Serial.print(", z: ");
//  Serial.println(mag.z());

  // Get magnetometer message
  vec = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu_mag.data[0] = vec.x();
  imu_mag.data[1] = vec.y();
  imu_mag.data[2] = vec.z();

  // Get calibration status
  bno.getCalibration(&imu_calib_status.system, &imu_calib_status.gyro, 
                     &imu_calib_status.accel, &imu_calib_status.mag);

  // Publish message
  mag_pub.publish(&imu_mag);
  imu_pub.publish(&imu_data);
  imu_status_pub.publish(&imu_calib_status);
  
  nh.spinOnce();
  delay(1);
}


