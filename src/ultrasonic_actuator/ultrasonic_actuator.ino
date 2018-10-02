
int incomingByte = 0;   // for incoming serial data
int LPWM = 10; // H-bridge leg 1 ->LPWM
int enL = 8; // H-bridge enable pin 1 -> L_EN


int RPWM = 11; // H-bridge leg 2 ->RPWM
int enR = 7; // H-bridge enable pin 2 -> R_EN

int a; // actuator position
int c = 0; // control position

int num_sections = 5;
int signal_max = 135;
int signal_min = 30;

int section_size = (signal_max - signal_min) / num_sections;



const int trigPin = 5;
const int echoPin = 6;
// defines variables
long duration;
int distance;






void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  pinMode(LPWM, OUTPUT); 
  pinMode(RPWM, OUTPUT); 
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);

  digitalWrite(enL, HIGH);
  digitalWrite(enR, HIGH);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication



  digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.34/2;

  
  int sensorValue = distance;

//  if (sensorValue > 0 && sensorValue <= 100) 
//  { a = 5;}
//  else if (sensorValue > 100 && sensorValue <= 200)
//  {a = 4; }
//  else if (sensorValue > 200 && sensorValue <= 300)
//  {a = 3; }
//  else if (sensorValue > 300 && sensorValue <= 400)
//  {a = 2; }
//  else if (sensorValue > 400 && sensorValue <= 500)
//  {a = 1; }
//  else
//  {a = 0 ;}

  if (sensorValue <= signal_min) {
    a = 0;
  }
  else if (sensorValue >= signal_max) {
    a = num_sections - 1;
  }
  else {
    for (int i = 0; i < num_sections; ++i) {
      if ((sensorValue > (i*section_size + signal_min)) && (sensorValue <= ((i + 1)*section_size + signal_min))) {
        a = i;
      }
    }
  }
}

  
  

void loop() {


  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
             
    c = incomingByte - 48;
    // Serial.println(c);
  }

  bool movein;
  bool moveout;


  digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.34/2;





   
  if (c < a){
    movein = true;
    moveout = false;
  }
  else if (c > a)
  {
    movein = false;
    moveout = true;
  }
  else
  {
    movein = false;
    moveout = false;
  } 
  
  int sensorValue =   distance;

//  if (moveout)
//  {
//    if (sensorValue < 50)
//    { a = 5;}
//    else if (sensorValue < 150)
//    {a = 4; }
//    else if (sensorValue < 250)
//    {a = 3; }
//    else if (sensorValue < 350)
//    {a = 2; }
//    else if (sensorValue < 450)
//    {a = 1; }
//    else
//    {a = 0 ;}
//  }
//  else if (movein)
//  {
//    if (sensorValue > 550)
//    { a = 0;}
//     else if (sensorValue > 450)
//    {a = 1; }
//    else if (sensorValue > 350)
//    {a = 2; }
//    else if (sensorValue > 250)
//    {a = 3; }
//    else if (sensorValue > 150)
//    {a = 4; }
//    else
//    {a = 5;}
//  }
//  else
//  {
//    if (sensorValue > 0 && sensorValue <= 100) 
//    { a = 5;}
//    else if (sensorValue > 100 && sensorValue <= 200)
//    {a = 4; }
//    else if (sensorValue > 200 && sensorValue <= 300)
//    {a = 3; }
//    else if (sensorValue > 300 && sensorValue <= 400)
//    {a = 2; }
//    else if (sensorValue > 400 && sensorValue <= 500)
//    {a = 1; }
//    else
//    {a = 0 ;}
//  }


  if (moveout) {
    for (int i = (num_sections - 1); i > -1; --i) {
      if (sensorValue > (i*section_size + section_size/2 + signal_min)) {
        a = i;
        break;
      }
      if (sensorValue < section_size/2 + signal_min) {
        a = 0;
        break;
      }
    }
  }
  else if (movein) {
    for (int i = 0; i < num_sections; ++i) {
      if (sensorValue < i*section_size + section_size/2 + signal_min) {
        a = i;
        break;
      }
      if (sensorValue > (num_sections - 1)*section_size + section_size/2 + signal_min) {
        a = num_sections - 1;
        break;
      }
    }
  }
  else {
    for (int i = 0; i < num_sections; ++i) {
      if ((sensorValue > (i*section_size + signal_min)) && (sensorValue <= ((i + 1)*section_size + signal_min))) {
        a = i;
        break;
      }
      if (sensorValue <= signal_min) {
        a = 0;
        break;
      }
      else if (sensorValue >= signal_max) {
        a = num_sections - 1;
        break;
      }
    }
  }
  
  Serial.print(a);
  Serial.print(":");
  Serial.print(c);
  Serial.print(", ");
  Serial.println(sensorValue);
  
  
  if (a > c)
  {
    analogWrite(LPWM,255);
    digitalWrite(RPWM,LOW);
  }
  else if (a < c)
  {
    analogWrite(RPWM,255); //pwm value
    digitalWrite(LPWM, LOW);}
  else 
  {
    digitalWrite(RPWM,LOW);
    digitalWrite(LPWM,LOW);
  }
  delay(30);
}




