#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimerOne.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

volatile bool collect = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 

  delay(100);

  Timer1.initialize(200000);
  Timer1.attachInterrupt(set_flag);

  if(!bno.begin()) {
    Serial.print("No BNO055 detected.");
    while(1);
  }
  
  delay(100);
    
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);

}

void set_flag() {
  collect = true;
}

void loop() {

  if (collect) {

    imu::Quaternion quat = bno.getQuat(); 
    imu::Vector<3> ang = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    float imuData[] = { quat.w(), quat.x(), quat.y(), quat.z(),
                        ang.x(), ang.y(), ang.z(),
                        lin.x(), lin.y(), lin.z()};

    Serial.print("\001");
    Serial.write((const uint8_t *)&imuData,sizeof(float) * 10);
        
    collect = false;
  }

}


