
#include <ros.h>
#include <std_msgs/Int16.h>

int ir_1 = 2;
int ir_2 = 4;
int r_counter;
int l_counter;
float distance;
float cycle;
int oldenc_1 = 0;
int oldenc_2 = 0;

// Set up ROS objects
ros::NodeHandle nh;
std_msgs::Int16 lwheel_msg;
std_msgs::Int16 rwheel_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_msg);
ros::Publisher rwheel_pub("rwheel", &rwheel_msg);

void setup() {
  // Set pins for encoder IR sensors
  pinMode(ir_1, INPUT);
  pinMode(ir_2, INPUT);

  // Begin ROS Node
  nh.initNode();
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  Serial.begin(57600);
}

void loop() {
  // Read if right wheel encoder sees a new segment
  int ir1 = digitalRead(ir_1);
  
  if (ir1 != oldenc_1)
  { 
//      // Sets counter values from 0-11
//      if (r_counter == 11){
//        r_counter = 0;
//        publish_right(r_counter);
//      }
//      else 
//      {
//        r_counter++;
//        publish_right(r_counter);
//      }
//  
//      if (l_counter == 0){
//        l_counter = 11;
//        publish_left(l_counter);
//      }
//      else
//      {
//        l_counter--;
//        publish_left(l_counter);
//      }
    // Publish pure 16-bit Int, overflow is considered in the Python code
    r_counter++;
    publish_right(r_counter);
    l_counter++;
    publish_left(l_counter);
  }

  float cycle = r_counter / 12;
  distance = (r_counter / 6) * 4 * 3.14;
  float l_counter = -(r_counter);
  
  oldenc_1 = ir1;

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

