/* This code handles all of the sensors that must be read or written
   to by the Arduino. Ultimately, we would like all sensor reading
   and manipulation to be performed through the Raspberry Pi, but 
   this is our best method for now due to the fact that the Pi does
   does not take analog inputs and using only 3.3V logic. The Arduino
   MEGA allows for 16 analog inputs and uses 5V logic.
   
   The linear regression to obtain the distance from the IR signal reading
   was obtained from "20-150cm Inverse Plot Data.csv" in /config of this 
   package. Fit the '.csv' data in the file to the model y = m*x + b, where
   x is Voltage and y is (1/cm). Remove the last two points from the curve
   since these are in the non-linear portion where the signal decreases
   again after going beyond the minimum distance reading.
   
   Sensors
   Proximity Sensors
     -12 IR distance sensors
     -3 Ultrasonic sensors (US)
   Linear Actuator Feedback
     -1 Ultrasonic sensor
   Steering Sensor
     -1 Potentiometer?
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <sensors/ProximitySensorArray.h>

// Proximity IR Sensors
// Linear regression parameters obtained by plotting data from Datasheet
const float M = 49.8860143139;
const float B = 0.1898448252;
// IR Pins
const int NUM_IR_SENSORS = 12; // if you change the number of sensors, you need to change the file sensors/msgs/ProximitySensorArray.msg to have a matching array length.
const int IR_SENSOR1 = A0;
const int IR_SENSOR2 = A1;
const int IR_SENSOR3 = A2;
const int IR_SENSOR4 = A3;
const int IR_SENSOR5 = A4;
const int IR_SENSOR6 = A5;
const int IR_SENSOR7 = A6;
const int IR_SENSOR8 = A7;
const int IR_SENSOR9 = A8;
const int IR_SENSOR10 = A9;
const int IR_SENSOR11 = A10;
const int IR_SENSOR12 = A11;
const int IR_SENSORS[NUM_IR_SENSORS] = {IR_SENSOR1, IR_SENSOR2, IR_SENSOR3, IR_SENSOR4, IR_SENSOR5,
                                        IR_SENSOR6, IR_SENSOR7, IR_SENSOR8, IR_SENSOR9, IR_SENSOR10,
                                        IR_SENSOR11};
// Use this pin to check the maximum voltage currently available, 
// this voltage can be used to adjust the IR sensor values to give 
// a more accurate distance.
//const int VOLTAGE_READ = A12;

// Proximity US Sensors
const int NUM_US_SENSORS = 3; // if you change the number of sensors, you need to change the file sensors/msgs/ProximitySensorArray.msg to have a matching array length.
const int TRIG_PIN1 = 22;
const int ECHO_PIN1 = 23;
const int TRIG_PIN2 = 24;
const int ECHO_PIN2 = 25;
const int TRIG_PIN3 = 26;
const int ECHO_PIN3 = 27;
const int US_SENSORS[NUM_US_SENSORS][2] = {{TRIG_PIN1, ECHO_PIN1}, {TRIG_PIN2, ECHO_PIN2}, {TRIG_PIN3, ECHO_PIN3}};

// Linear Actuator Feedback
// Sensor Pins
const int TRIG_PIN_LA = 28;
const int ECHO_PIN_LA = 29;
// Motor Driver Pins
const int LPWM = 10; // H-bridge leg 1, this should pull the actuator in
const int L_EN = 8; // H-bridge enable pin 1
const int RPWM = 11; // H-bridge leg 2, this should push the actuator out
const int R_EN = 7; // H-bridge enable pin 2
// Controller Parameters
int num_targets = 5; // number of target values the actuator can move to, this discretizes the positioning
                     // needs to be at least 2, 1 for each endpoint
int num_sections = num_targets - 1; // number of divisions, based on target points
int signal_max = 135; // distance at max stroke length (mm)
int signal_min = 30; // distance at min stroke length (mm)
int section_size = (signal_max - signal_min) / num_sections; // to the nearest mm
int desired_position = 0; // desired actuator position
int current_position = signal_min; // current position of the actuator as read from the US sensor (mm)
int noise_threshold = 20; // (mm) the threshold value that the error (+ or -) must be greater than to force the actuator to move without being explicitly commanded by the brake_fraction
float fraction_section_size = 1.0/num_sections; // used to determine the section the brake fraction should fall into
float brake_fraction = 0; // percentage of stroke length to move to, this value is converted into the desired section
                         // this value is read in by a ROS subscriber
boolean update_position = false; // turns true when the current error goes outside the noise range or a new command has been sent.
boolean direction = 0; // 1 = move outward, 0 = move inward

// Axle Sensor
// *currently unknown*

// Define ROS Variables
ros::NodeHandle nh;
sensors::ProximitySensorArray distances;
ros::Publisher proximity_pub("proximity_sensors", &distances);

// Callback function which updates the global variable 'desired_position
// brake_fraction is converted into a desired position which is referenced in the loop() function
void updateDesiredPosition(const std_msgs::Float32& msg);
ros::Subscriber<std_msgs::Float32> brake_fraction_sub("controls/brake_fraction", &updateDesiredPosition);
// TO-DO: Add subscribers for the data you need to send out control signals to the forklift

void setup()
{
  // Setup ROS
  nh.initNode();
  nh.advertise(proximity_pub);
  
  // Setup Pins
  // Proximity IR
  // *these are analog pins and do not need to be setup
  
  // Proximity US
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);
  
  // Actuator Feedback
  pinMode(TRIG_PIN_LA, OUTPUT);
  pinMode(ECHO_PIN_LA, INPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  
  // Axle Sensor
  //*currently unknown*
}

void loop()
{
  //===== Proximity Sensors =====//
  // Read proximity sensors
  // IR Distance
  for (int i = 0; i < NUM_IR_SENSORS; ++i) {
    distances.ir_data[i] = getIRDistance(IR_SENSORS[i]);
  }
  
  // Ultrasonic Distance
  for (int i = 0; i < NUM_US_SENSORS; ++i) {
    distances.us_data[i] = getUSDistance(US_SENSORS[i][0], US_SENSORS[i][1]);
  }
  
  // Publish values
  distances.seq++;
  distances.stamp = nh.now();
  proximity_pub.publish(&distances);
  //===== Proximity Sensors =====//
  
  //===== Linear Actuator Control =====//
  // Read position
  current_position = getUSDistance(TRIG_PIN_LA, ECHO_PIN_LA);
  
  // Update desired section based on brake fraction [0 -> 1]
  
  
  // Check if the position error is beyond the noise threshold, if so move the actuator
  // The direction you are traveling will determine the stopping criteria
  int position_error = current_position - desired_position;
  
  if (position_error > noise_threshold) {
    direction = 0; // move inward
    update_position = true;
  }
  if (position_error < -noise_threshold) {
    direction = 1; // move outward
    update_position = true;
  }
  
  // Control power output to actuator
  if (update_position) {
    // Check if the target has been reached
    if (direction) { // currently moving outward
      if (position_error >= 0) { // point is reached or passed
        digitalWrite(RPWM, LOW);
        digitalWrite(LPWM, LOW);
        update_position = false;
      }
      else { // point has not been reached
        digitalWrite(LPWM, LOW);
        analogWrite(RPWM, 255);
      }
    }
    else { // currently moving inward
      if (position_error <= 0) { // point is reached or passed
        digitalWrite(RPWM, LOW);
        digitalWrite(LPWM, LOW);
        update_position = false;
      }
      else { // point has not been reached
        digitalWrite(RPWM, LOW);
        analogWrite(LPWM, 255);
      }
    }
  }
  
  //===== Linear Actuator Control =====//
  
  //===== Axle Sensor =====//
  //===== Axle Sensor =====//
}

// Returns the distance given from the IR sensor
// Performs the regression and return the distance in (m)
float getIRDistance(int ir_pin)
{
  // Find distance by applying linear regression function
  // y = m*x + b
  // y = Distance in cm^-1
  // x = Signal Voltage
  int max_analog = 1024; // read from VOLTAGE_READ here to get the max signal
  int signal = analogRead(ir_pin);
  float signal_voltage = signal*(5.0/max_analog); // assumes 5V power to sensor
  float inverse_distance = M*signal_voltage + B;
  float distance_m = 1/(100*inverse_distance); // convert to meters
  
  return distance_m;
}

// Returns the distance read by the Ultrasonic sensor
// Distance returned is in (mm) as an integer
int getUSDistance(int trig_pin, int echo_pin)
{
  // Send out sound pulse
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // Set the trig_pin to HIGH for 10 micro seconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  
  // Read the echo_pin, which turns HIGH for the length of time the sound 
  // wave traveled (in microseconds)
  long duration = pulseIn(echo_pin, HIGH); // microseconds
  
  // Calculate the distance traveled ***(mm)***
  // Here we use milimeters to be at an appropriate integer resolution
  // for the actuator regions
  float speed_of_sound = 0.343; // (mm/microsecond)
  int distance = (duration / 2.0)*speed_of_sound; // divide duration by 2 since the sound wave goes out then back, so it travels twice the distance we are looking for
}

void updateDesiredPosition(const std_msgs::Float32& msg)
{
  // Read in new brake command
  brake_fraction = msg.data;
  
  // Convert into a position along the stroke length (based on number of targets)
  
  // Find error and determine direction
  current_position = getUSDistance(TRIG_PIN_LA, ECHO_PIN_LA);
  int position_error = current_position - desired_position;
  
  if (position_error > 0) {
    direction = 0; // move inward
  }
  else {
    direction = 1; // move outward
  }
  
  // Tell actuator it needs to update its position
  update_position = true;
}
