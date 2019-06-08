#!/usr/bin/env python3

'''
This code reads the velocity from the the canbus signal from the forklift. The
message of interest is 0x388 bytes 2 and 3 (with byte 2 being the MSB of the
16bit signed int). The value is also converted to a predicted speed in KMH.
This data is then published on a ROS topic.
'''

import rospy
from std_msgs.msg import Float64, Int16
import binascii # for 'hexlify'
import can # requires Python3


class VelocityNode:
    def __init__(self):
        # Set up ROS Parameters and Objects
        rospy.init_node("velocity_node")
        self.velocity_pub = rospy.Publisher("velocity_node/velocity", Float64, queue_size=10)
        self.velocity_kmh_pub = rospy.Publisher("velocity_node/velocity_kmh", Float64, queue_size=10)
        self.canbus_pub = rospy.Publisher("velocity_node/canbus_velocity", Int16, queue_size=10)
        self.velocity_msg = Float64()
        self.velocity_kmh_msg = Float64()
        self.canbus_msg = Int16()

        # Velocity parameters
        self.traction_wheel_factor = 8520.

        # Set up canbus connection
        self.bus_name = rospy.get_param("~/bus_name", "slcan0")
        self.can_id = '0x388' # CAN ID for the message containing velocity data
        # the order to combine the raw data bytes in
        # e.g. [2,3] means get byte 2 as the MSB with byte 3 as the LSB and combine them
        # to get a 16 bit integer (four numbers in the array would produce 32 bit, etc.)
        self.byte_order = [2,3]
        self.signed_int = True

        # Start the canbus connection
        self.canbus = can.Bus(channel=self.bus_name, bustype="socketcan")

    def spin(self):
        while not rospy.is_shutdown():
            # Read canbus message
            msg = self.canbus.recv(1)

            if (msg):
                # Check for matching CAN ID
                if (msg.arbitration_id == int(self.can_id,16)):
                    data_str = binascii.hexlify(msg.data).decode('ascii').upper()
                    data_array = [data_str[i:i+2] for i in range(0, len(data_str), 2)]

                    # Get value from raw bytes
                    bytes = []
                    for byte_num in self.byte_order:
                        bytes.append(data_array[byte_num])
                    byte_str = "".join(bytes)
                    value = int(byte_str, 16)

                    # Use Two's Complement to turn signed ints to appropriate value
                    if (self.signed_int):
                        value = self.twos_comp(value, 16)

                    # Convert value to predicted velocity
                    velocity_kmh = 10.0*(value/self.traction_wheel_factor)
                    # Covnert to m/s
                    velocity = velocity_kmh/3.6

                    # Create messages
                    self.velocity_msg.data = velocity
                    self.velocity_kmh_msg.data = velocity_kmh
                    self.canbus_msg.data = value

                    # Publish value and velocity
                    self.velocity_pub.publish(self.velocity_msg)
                    self.velocity_kmh_pub.publish(self.velocity_kmh_msg)
                    self.canbus_pub.publish(self.canbus_msg)

    def twos_comp(self, value, bits):
        '''
        Calculates the two's complement of 'value' for a specified number of bits
        '''
        if (value & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            value = value - (1 << bits)        # compute negative value
        return value


if __name__ == '__main__':
    try:
        velocity_node = VelocityNode()
        velocity_node.spin()
    except rospy.ROSInterruptException:
        pass
