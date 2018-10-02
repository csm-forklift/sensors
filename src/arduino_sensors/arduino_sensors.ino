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
#include <sensors/ProximitySensorArray.h>

// Proximity IR Sensors
// Linear regression parameters obtained by plotting data from Datasheet
const float m = 49.8860143139;
const float b = 0.1898448252;
// IR Pins
const int num_ir_sensors = 12;
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
// Use this pin to check the maximum voltage currently available, 
// this voltage can be used to adjust the IR sensor values to give 
// a more accurate distance.
//const int VOLTAGE_READ = A12;

// Proximity US Sensors
const int TRIG_PIN1 = 22;
const int ECHO_PIN1 = 23;
const int TRIG_PIN2 = 24;
const int ECHO_PIN2 = 25;
const int TRIG_PIN3 = 26;
const int ECHO_PIN3 = 27;

// Linear Actuator Feedback
// Sensor Pins
const int TRIG_PIN4 = 28;
const int ECHO_PIN4 = 29;
// Motor Driver Pins
const int LPWM = 10; // H-bridge leg 1
const int L_EN = 8; // H-bridge enable pin 1
const int RPWM = 11; // H-bridge leg 2
const int R_EN = 7; // H-bridge enable pin 2
// Controller Parameters
int num_sections = 5; // number of divisions for the actuator stroke
int signal_max = 135; // distance at max stroke length (mm)
int signal_min = 30; // distance at min stroke length (mm)
int section_size = (signal_max - signal_min) / num_sections; // to the nearest mm
int pos_act; // actuator position
int des_act; // desired actuator position


// Axle Sensor
// *currently unknown*

// Define ROS Variables
ros::NodeHandle nh;
sensors::ProximitySensorArray distances;
ros::Publisher proximity_pub("proximity_sensors", &distances);
// TO-DO: Add subscribers for the data you need to send out control signals

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
  pinMode(TRIG_PIN4, OUTPUT);
  pinMode(ECHO_PIN4, INPUT);
  
  // Axle Sensor
}

void loop()
{
  //===== Proximity Sensors =====//
  // Read proximity sensors
  // IR Distance
  // Ultrasonic Distance
  // Publish values
  //===== Proximity Sensors =====//
  
  //===== Linear Actuator Control =====//
  // Read position
  // Check range
  // Control power output to actuator
  //===== Linear Actuator Control =====//
  
  //===== Axle Sensor =====//
  //===== Axle Sensor =====//
}

float getIRDistance(int ir_pin)
{
  // Find distance by applying linear regression function
  // y = m*x + b
  // y = Distance in cm^-1
  // x = Signal Voltage
  int max_analog = 1024; // read from VOLTAGE_READ here to get the max signal
  int signal = analogRead(ir_pin);
  float signal_voltage = signal*(5.0/max_analog); // assumes 5V power to sensor
  float inverse_distance = m*signal_voltage + b;
  float distance_m = 1/(100*inverse_distance); // convert to meters
  
  return distance_m;
}

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
  
  // Calculate the distance travel ***(mm)***
  // Here we use milimeters to be at an appropriate integer resolution
  // for the actuator regions
  float speed_of_sound = 0.343; // (mm/microsecond)
  int distance = (duration / 2.0)*speed_of_sound; // divide duration by 2 since the sound wave goes out then back, so it travels twice the distance we are looking for
}
