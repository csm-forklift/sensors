# The sensor_msg/Imu.h message is too large to send using an Arduino directly
# Instead I am creating is message which is simply a Float32 array (Arduino cannot
# handle 64 bits)
# The values should be published and interpreted in this format:
# data[0] = quaternion.w
# data[1] = quaternion.x
# data[2] = quaternion.y
# data[3] = quaternion.z
# data[4] = angular_velocity.x
# data[5] = angular_velocity.y
# data[6] = angular_velocity.z
# data[7] = linear_acceleration.x
# data[8] = linear_acceleration.y
# data[9] = linear_acceleration.z
float32[10] data
