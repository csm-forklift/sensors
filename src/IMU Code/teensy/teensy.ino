#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;

sensor_msgs::Imu imuData;
ros::Publisher imu_pub("imu_data",&imuData);

int ledPin = 13;

void auto_drive( const geometry_msgs::Twist& cmd_vel) {
  float values[2] {cmd_vel.linear.x, cmd_vel.angular.z};

  Serial2.print('\001');
  Serial2.write((uint8_t *)&values, sizeof(float) * 2);
  digitalWrite(ledPin, HIGH-digitalRead(ledPin));

}

void imu_reader() {
  if (Serial1.read() == '\001') {
    float imuArray[10];
        
    char *ptr = (char *)&imuArray;
  
    Serial1.readBytes(ptr, sizeof(float)*10);
  
    imuData.orientation.w = imuArray[0];
    imuData.orientation.x = imuArray[1];
    imuData.orientation.y = imuArray[2];
    imuData.orientation.z = imuArray[3];
  
    imuData.angular_velocity.x = imuArray[4];
    imuData.angular_velocity.y = imuArray[5];
    imuData.angular_velocity.z = imuArray[6];
  
    imuData.linear_acceleration.x = imuArray[7];
    imuData.linear_acceleration.y = imuArray[8];
    imuData.linear_acceleration.z = imuArray[9];
  
    imu_pub.publish(&imuData);
  }
}


void exc_state( const std_msgs::Int32& cur_state) {
  Serial3.print("\001");
  Serial3.write((const uint8_t *)&cur_state.data, sizeof(int));
  digitalWrite(ledPin, HIGH-digitalRead(ledPin));
}

ros::Subscriber<geometry_msgs::Twist>cmd_vel_sub("cmd_vel", &auto_drive);
ros::Subscriber<std_msgs::Int32>exc_state_sub("exc_state", &exc_state);

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  // Start ROS
  nh.initNode();

  nh.advertise(imu_pub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(exc_state_sub);

  pinMode(ledPin, OUTPUT);

  while (!nh.connected()) { nh.spinOnce(); }
}


void loop() {
  if (Serial1.available()) {
    imu_reader();    
  }

  nh.spinOnce();

}
