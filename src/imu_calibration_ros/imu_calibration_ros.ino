#include <ros.h>
#include <sensors/ImuCalibration.h>
#include <sensors/ImuCalibStatus.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */

/*=======================
 * Calibration Procedure
 *=======================
 * Accelerometer
 * - spin the IMU in a circle holding it at 45 deg increments for a few seconds
 * 
 * Gyro
 * - leave the IMU flat on a table for a few seconds
 * 
 * Magnetometer
 * - move the IMU in a figure 8 pattern
 */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ROS Objects
ros::NodeHandle nh;
sensors::ImuCalibration imu_calib;
ros::Publisher imu_calib_pub("imu/calibration", &imu_calib);
sensors::ImuCalibStatus imu_cal_status;
ros::Publisher imu_cal_status_pub("imu/status", &imu_cal_status);

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void publishCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    // Publish Status
    imu_cal_status.system = system;
    imu_cal_status.accel = accel;
    imu_cal_status.gyro = gyro;
    imu_cal_status.mag = mag;
    imu_cal_status_pub.publish(&imu_cal_status);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void publishSensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    imu_calib.data[0] = calibData.accel_offset_x;
    imu_calib.data[1] = calibData.accel_offset_y;
    imu_calib.data[2] = calibData.accel_offset_z;
    imu_calib.data[3] = calibData.accel_radius;
    imu_calib.data[4] = calibData.gyro_offset_x;
    imu_calib.data[5] = calibData.gyro_offset_y;
    imu_calib.data[6] = calibData.gyro_offset_z;
    imu_calib.data[7] = calibData.mag_offset_x;
    imu_calib.data[8] = calibData.mag_offset_y;
    imu_calib.data[9] = calibData.mag_offset_z;
    imu_calib.data[10] = calibData.mag_radius;
    // Publish message
    imu_calib_pub.publish(&imu_calib);
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
void setup(void)
{
    nh.initNode();
    nh.advertise(imu_calib_pub);
    nh.advertise(imu_cal_status_pub);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    
    Serial.begin(57600);
    delay(1000);
    
    /* Initialise the sensor */
    if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        while (1) {
          digitalWrite(13, HIGH);
          delay(500);
          digitalWrite(13, LOW);
          delay(500);
        }
    }

    delay(500);
    digitalWrite(13, LOW);
}

void loop() {

    /* Optional: Display calibration status */
    publishCalStatus();

    /* Optional: Display sensor offsets */
    adafruit_bno055_offsets_t calibrationData;
    bno.getSensorOffsets(calibrationData);
    publishSensorOffsets(calibrationData);

    /* Wait the specified delay before requesting new data */
    delay(BNO055_SAMPLERATE_DELAY_MS);
    
    nh.spinOnce();
}
