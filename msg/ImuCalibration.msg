# Message containing the IMU calibration parameters
#
# Data Format
# data[0] = accel_offset_x
# data[1] = accel_offset_y
# data[2] = accel_offset_z
# data[3] = accel_radius
# data[4] = gyro_offset_x
# data[5] = gyro_offset_y
# data[6] = gyro_offset_z
# data[7] = mag_offset_x
# data[8] = mag_offset_y
# data[9] = mag_offset_z
# data[10] = mag_radius

int16[11] data
