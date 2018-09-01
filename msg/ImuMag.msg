# Sends the x,y,z values for the magnetic field vector as a simple
# array to save memory on the Arduino. This message replaces the
# geometry_msgs/Vector3Stamped message type.
# The values should be published and interpreted in this format
# data[0] = vector.x
# data[1] = vector.y
# data[2] = vector.z
float32[3] data
