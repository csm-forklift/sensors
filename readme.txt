-----------------
Notes on IMU Data
-----------------
When publishing the IMU data on the Arduino, we choose to publish the data as an array of floats with 10 elements, (first 4 for quaternion, next 3 for angular velocity, and next 3 for linear acceleration). There is not enough memory on the Arduino to safely use the sensor_msg/Imu message, this is why we choose to use an array of floats. Arduino is picky about using arrays, so we had to make our own custom message that is of type float32[10], then we subscribe to that topic and convert it to the sensor_msg/Imu message type in a separate node.
