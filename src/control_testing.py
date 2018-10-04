#!/usr/bin/env python

''' This code is meant to be used for testing the various control codes for
debugging purposes.'''

import rospy
from std_msgs.msg import Float32
from sensors.msg import ProximitySensorArray


class ControlTesting():
    def __init__(self):
        # Begin node
        rospy.init_node("control_testing")

        # Set publishers and subscribers, try to group by control system so that
        # way you can comment them out in blocks

        # Arduino sensors
        self.proximity_sub = rospy.Subscriber("proximity_sensors", ProximitySensorArray, self.proximity_callback)

        # Brake Actuator
        self.brake_pub = rospy.Publisher("controls/brake/input", Float32, queue_size=10)
        self.brake_sub = rospy.Subscriber("controls/brake/position", Float32, self.brake_callback)

        # Accelerator signal
        self.accelerator_pub = rospy.Publisher("controls/accelerator/input", Float64, queue_size=10)

    def update(self):
        fraction = raw_input("Enter drive ([-1, 1] -1 = full brake, 1 = full throttle): ")

        # Check if the value is for braking or throttle
        if (fraction < 0): # Brake
            fraction_msg = Float32()
            fraction = fabs(fraction)
            if (fraction > 1.):
                fraction = 1.
            if (fraction < 0.):
                fraction = 0.
            fraction_msg.data = float(fraction)
            self.brake_pub.publish(fraction_msg)
        else: # Accelerate
            fraction_msg = Float64()
            if (fraction > 1.):
                fraction = 1.
            if (fraction < 0.):
                fraction = 0.
            fraction_msg.data = float(fraction)
            self.accelerator_pub.publish(fraction_msg)


    def spin(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def proximity_callback(self, msg):
        pass
        #print msg

    def brake_callback(self, msg):
        pass
        #print msg



if __name__ == "__main__":
    try:
        control_test = ControlTesting()
        control_test.spin()
    except rospy.ROSInterruptException:
        pass
