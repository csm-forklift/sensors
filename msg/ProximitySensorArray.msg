# Simple array of distances provided by the proximity sensors.
# Allows the Arduino to publish required data with less memory cost.
#
# The ultrasonic sensors (us_data) do not have to be used and can either be edited
# out of this code or simply not stored.
# You must take note as to which IR sensor corresponds to which data element.

float32[12] ir_data
float32[3] us_data
