# Simple array of distances provided by the proximity sensors.
# Allows the Arduino to publish required data with less memory cost.
#
# The ultrasonic sensors (us_data) do not have to be used and can either be edited
# out of this code or simply not stored.
# You must take note as to which IR sensor corresponds to which data element.

# Header data. For some reason it will not compile if you use a real std_msgs/Header message
uint32 seq
time stamp

float32[12] ir_data
float32[3] us_data
