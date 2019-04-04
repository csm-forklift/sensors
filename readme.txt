-----------------
Notes on IMU Data
-----------------
When publishing the IMU data on the Arduino, we choose to publish the data as an array of floats with 10 elements, (first 4 for quaternion, next 3 for angular velocity, and next 3 for linear acceleration). There is not enough memory on the Arduino to safely use the sensor_msg/Imu message, this is why we choose to use an array of floats. Arduino is picky about using arrays, so we had to make our own custom message that is of type float32[10], then we subscribe to that topic and convert it to the sensor_msg/Imu message type in a separate node.

The IMU absolute orientation has an issue where it starts out using relative 
angles until the magnetometer gets calibrated. It then realigns itself with the
magnetic field for a correct absolute orientation. This results in a big jump in
orientation. To help avoid this (you could just use the regular IMU mode that 
considers relative orientation only) if you want to keep using the magnetometer 
for absolute orientation data, you can edit the Adafruit header file 
"Adafruit_BNO055.h" found in "[sketchbook location]/libraries/Adafruit_BNO055/" 
to start the device off in the "FMC" (Fast Magnetometer Calibration) mode. In 
the "Adafruit_BNO055.h" file, locate the function prototype for "begin()" and 
make sure it contains "adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF". You
want the "mode" variable to default to something that is NOT 
"OPERATION_MODE_NDOF_FMC_OFF" (which does not do the fast calibration). The 
option listed above still fuses all sensors while doing the fast calibration and
giving an orientation that will not shift by a large value once the magnetometer
is fully calibrated.
