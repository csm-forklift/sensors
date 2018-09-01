// ROS Includes
#include <ros.h>
#include <encoders/ImuArray.h>
#include <std_msgs/Int16.h>

// IMU Includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Encoder pins and variables
int ir_1 = 2;
int ir_2 = 4;
int r_counter;
int l_counter;
float distance;
float cycle;
int oldenc_1 = 0;
int oldenc_2 = 0;

// ROS Objects
ros::NodeHandle nh;
// encoder
std_msgs::Int16 rwheel_msg;
std_msgs::Int16 lwheel_msg;
ros::Publisher rwheel_pub("enc/wheel_right_ticks", &rwheel_msg);
ros::Publisher lwheel_pub("enc/wheel_left_ticks", &lwheel_msg);
// IMU
encoders::ImuArray imuData;
ros::Publisher imu_pub("imu/raw_data", &imuData);

// IMU Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Quaternion quat;
imu::Vector<3> ang;
imu::Vector<3> lin;

void setup() {
  // Set pins for encoder IR sensors
  pinMode(ir_1, INPUT);
  pinMode(ir_2, INPUT);
  
  // Start ROS Node
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(rwheel_pub);
  nh.advertise(lwheel_pub);
  Serial.begin(57600);
  delay(100);

  // Wait for BNO055 to begin before continuing
  if (!bno.begin()) {
    //Serial.println("No BNO055 detected.");
    while(1);
  }
}

void loop() {
  //===== Encoder Section =====//
  // Read if right wheel encoder sees a new segment
  int ir1 = digitalRead(ir_1);
  
  if (ir1 != oldenc_1)
  { 
    // Publish pure 16-bit Int, overflow is considered in the Python code
    r_counter++;
    l_counter++;
  }

  publish_right(r_counter);
  publish_left(l_counter);
  
  oldenc_1 = ir1;
  //===== Encoder Section =====//

  //===== IMU Section =====//
  // Get IMU data from sensor
  quat = bno.getQuat(); 
  ang = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Convert to IMU array message
  imuData.data[0] = quat.w();
  imuData.data[1] = quat.x();
  imuData.data[2] = quat.y();
  imuData.data[3] = quat.z();
  imuData.data[4] = ang.x();
  imuData.data[5] = ang.y();
  imuData.data[6] = ang.z();
  imuData.data[7] = lin.x();
  imuData.data[8] = lin.y();
  imuData.data[9] = lin.z();

  // Publish message
  imu_pub.publish(&imuData);
  //===== IMU Section =====//
  
  nh.spinOnce();
}

void publish_right(int r_counter)
{
  rwheel_msg.data = r_counter;
  rwheel_pub.publish(&rwheel_msg);
}

void publish_left(int l_counter)
{
  lwheel_msg.data = l_counter;
  lwheel_pub.publish(&lwheel_msg);
}
