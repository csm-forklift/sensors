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
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
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
const int LPWM = 30; // H-bridge leg 1, this should pull the actuator in
const int L_EN = 32; // H-bridge enable pin 1
const int RPWM = 31; // H-bridge leg 2, this should push the actuator out
const int R_EN = 33; // H-bridge enable pin 2
// Controller Parameters
int num_targets = 5; // number of target values the actuator can move to, this discretizes the positioning
                     // needs to be at least 2, 1 for each endpoint
int num_sections = num_targets - 1; // number of divisions, based on target points
int signal_max = 170; // distance at max stroke length (mm)
int signal_min = 50; // distance at min stroke length (mm)
int section_size = (signal_max - signal_min) / num_sections; // to the nearest mm
int desired_position = 0; // desired actuator position
int current_position = signal_min; // current position of the actuator as read from the US sensor (mm)
int noise_threshold = 20; // (mm) the threshold value that the error (+ or -) must be greater than to force the actuator to move without being explicitly commanded by the brake_input
float brake_input = 0; // percentage of stroke length to move to, this value is converted into the desired section
                         // this value is read in by a ROS subscriber
float position_fraction = 0; // this is the current position converted back into a fraction to publish back out for feedback
int current_target = 0; // used to determine whether we need to move or not
int previous_target = 0;
boolean update_position = false; // turns true when the current error goes outside the noise range or a new command has been sent.
boolean direction = 0; // 1 = move outward, 0 = move inward
// Values for storing a window of data and performing an average to smooth out the noise
const int WINDOW_SIZE = 10;
const float ALPHA = 0.5; // learning weight for exponentially weighted average
int position_window[WINDOW_SIZE] = {0};

// Axle Sensor
// *currently unknown*

// Define ROS Variables
ros::NodeHandle nh;
sensors::ProximitySensorArray distances;
ros::Publisher proximity_pub("proximity_sensors", &distances);

// Callback function which updates the global variable 'desired_position
// brake_input is converted into a desired position which is referenced in the loop() function
void updateDesiredPosition(const std_msgs::Float32& msg);
ros::Subscriber<std_msgs::Float32> brake_input_sub("controls/brake/input", &updateDesiredPosition);
std_msgs::Float32 position_msg;
ros::Publisher brake_position_pub("controls/brake/position", &position_msg);
std_msgs::Int32 signal_msg;
ros::Publisher brake_signal_pub("controls/brake/signal", &signal_msg);
// FIXME: debugging output message
std_msgs::String output_msg;
ros::Publisher output_message_pub("debug/output", &output_msg);
// TO-DO: Add subscribers for the data you need to send out control signals to the forklift

void setup()
{
  // Setup ROS
  nh.initNode();
  nh.advertise(proximity_pub);
  nh.advertise(brake_position_pub);
  nh.advertise(brake_signal_pub);
  nh.advertise(output_message_pub);
  nh.subscribe(brake_input_sub);
  
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
  
  // Testing Code
  //actuatorLimits(LPWM, RPWM);
}

void loop()
{
  //===== Proximity Sensors =====//
  // Read proximity sensors
  // IR Distance
  for (int i = 0; i < NUM_IR_SENSORS; ++i) {
    distances.ir_data[i] = getIRDistance(IR_SENSORS[i]);
  }
  
//  // Ultrasonic Distance (comment this out if you do not use US sensors for proximity)
//  for (int i = 0; i < NUM_US_SENSORS; ++i) {
//    distances.us_data[i] = getUSDistance(US_SENSORS[i][0], US_SENSORS[i][1]);
//  
//  }
  
  // Publish values
  distances.seq++;
  distances.stamp = nh.now();
  proximity_pub.publish(&distances);
  //===== Proximity Sensors =====//
  
  //===== Linear Actuator Control =====//
  // Read position
  current_position = getUSDistance(TRIG_PIN_LA, ECHO_PIN_LA);
  // Get exponentially weighted average to smooth out data
  current_position = expWeightedAverage(current_position);
  signal_msg.data = current_position;
  brake_signal_pub.publish(&signal_msg);
  
  // Convert current position into a fraction [0 -> 1] for publishing
  position_fraction = (float)(current_position - signal_min) / (float)(signal_max - signal_min);
  position_msg.data = position_fraction;
  brake_position_pub.publish(&position_msg);
  
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
    
    // FIXME: Test where update is going wrong
    String message = "Currently Updating " + String(direction) + ", " + String(current_position) + " -> " + String(desired_position);
    output_msg.data = message.c_str();
    //output_message_pub.publish(&output_msg);
    
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
  
  nh.spinOnce();
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
  float speed_of_sound = 0.34; // (mm/microsecond)
  int distance = (duration / 2.0)*speed_of_sound; // divide duration by 2 since the sound wave goes out then back, so it travels twice the distance we are looking for
  
  return distance;
}

void updateDesiredPosition(const std_msgs::Float32& msg)
{
  // Read in new brake command
  // Bound value between 0 and 1
  brake_input = msg.data;
  if (brake_input > 1.0) {
    brake_input = 1.0;
  }
  else if (brake_input < 0.0) {
    brake_input = 0.0;
  }
  
  // Convert into a position along the stroke length (based on number of targets)
  // (find nearest target number, then use target number to get position)
  // Targets are labelled like so:
  //       0     1     2     3          num_sections
  //       |-----|-----|-----|----- ... -----|
  //fully-retracted                    fully-extended
  float brake_conversion = brake_input*num_sections;
  float upper_error = abs(((int) brake_conversion + 1) - brake_conversion);
  float lower_error = abs(brake_conversion - ((int) brake_conversion));

  // Update target and check if we need to move
  if (upper_error < lower_error) { // round upward
    current_target = ((int) brake_conversion + 1);
  }
  else { // round down
    current_target = ((int) brake_conversion);
  }
  desired_position = current_target*section_size + signal_min;
  
  // Find error and determine direction
  current_position = getUSDistance(TRIG_PIN_LA, ECHO_PIN_LA);
  current_position = expWeightedAverage(current_position);
  int position_error = current_position - desired_position;
  
  if (position_error > 0) {
    direction = 0; // move inward
  }
  else {
    direction = 1; // move outward
  }
  
  // FIXME: check that values in this function are correct
  String message = "brake_conversion: " + String(brake_conversion, 3) + ", " +
                   "upper_error: " + String(upper_error, 3) + ", " +
                   "lower_error: " + String(lower_error, 3) + ", " +
                   "target: " + String(current_target) + ", " +
                   "current_position: " + String(current_position) + ", " +
                   "desired_position: " + String(desired_position) + ", " +
                   "position_error: " + String(position_error) + ", " +
                   "direction: " + String(direction);
  output_msg.data = message.c_str();
  output_message_pub.publish(&output_msg);
  
  // Tell actuator it needs to update its position
  if (current_target != previous_target) {
    update_position = true;
    previous_target = current_target;
  }
}

int expWeightedAverage(int value)
{
  float average = 0;
  // Update window array and compute average
  for (int i = 0; i < (WINDOW_SIZE - 1); ++i) {
    position_window[i] = position_window[i+1];
    if (i == 0) {
      average = position_window[i];
    }
    else {
      average = ALPHA*position_window[i] + (1 - ALPHA)*average;
    }
  }
  position_window[WINDOW_SIZE - 1] = value;
  average = ALPHA*value + (1 - ALPHA)*average;

  return average;
}

void actuatorLimits(int lpwm, int rpwm)
{
  // Push actuator out and wait 5 seconds (3 seconds for extension)
  digitalWrite(lpwm, LOW);
  analogWrite(rpwm, 255);
  delay(8000);
  
  // Pull actuator in and stop
  digitalWrite(rpwm, LOW);
  analogWrite(lpwm, 255);
}
