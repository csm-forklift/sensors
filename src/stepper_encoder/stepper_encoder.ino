/**
 * This code reads stepper motor encoder ticks and publishes a boolean indicating
 * whether the motor is moving or not.
 * 
 * The rate at which it checks can be adjusted by changing the loop delay time.
 * 
 * Pin Layout
 * ----------
 * Arduino         Encoder
 * D2 (interrupt)  Pin A (white)
 * D3              Pin B (Brown)
 * 5V              5V (Orange)
 * GND             GND (Black)
 */

// ROS Includes
#include <ros.h>
#include <std_msgs/Bool.h>

//===== Encoder Values =====//
#define encoder_pinA 2
#define encoder_pinB 3
volatile int counter = 0;
int prev_counter = 0;
int repeats = 0;
bool moving = false; // indicates whether the motor is moving or not

//===== Parameters =====//
const int DELAY_TIME = 32; // ms
// Number of times the counter value must remain the same 
// before considering the motor to be stopped.
const int NUM_REPEATS = 3;

//===== ROS Objects =====//
ros::NodeHandle nh;
std_msgs::Bool is_moving;
ros::Publisher moving_pub("/steering_node/motor/is_moving", &is_moving);

void setup() {
  // Set up ROS
  nh.initNode();
  nh.advertise(moving_pub);

  // Set up pins
  pinMode(encoder_pinA, INPUT);
  pinMode(encoder_pinB, INPUT);

  // Use Interrupt pin
  attachInterrupt(digitalPinToInterrupt(encoder_pinA), isr, RISING); // ISR = Interrupt Service Routine

  // Begin serial connection
  Serial.begin(57600);
}

void loop() {
  // Check if counter is the same as the previous value
  if (counter == prev_counter) {
    repeats++;
    if (repeats > NUM_REPEATS) {
      moving = false;
    }
    else {
      moving = true;
    }
  }
  else {
    repeats = 0;
    prev_counter = counter;
    moving = true;
  }

  // Publish message
  is_moving.data = moving;
  moving_pub.publish(&is_moving);

  nh.spinOnce();
  delay(DELAY_TIME); 
}

void isr() {
  int channel_A = digitalRead(encoder_pinA);
  int channel_B = digitalRead(encoder_pinB);

  if (channel_A == channel_B) {
    counter ++;
  }
  else {
    counter--;
    delay(100);
    
  }
}
