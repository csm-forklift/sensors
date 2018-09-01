# IMU Calibration Status
# Values go from 0..3
# 0 means no calibration, data should not be trusted
# 3 means calibration is complete
# Values are updated as the IMU is actively moved
# 
# Data
# system: full system reading (reject data if this value is 0)
# accel: accelerometer
# gyro: gyroscope
# mag: magnetometer

int8 system
int8 accel
int8 gyro
int8 mag
