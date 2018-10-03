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
        self.brake_pub = rospy.Publisher("controls/brake/desired", Float32, queue_size=10)
        self.brake_sub = rospy.Subscriber("controls/brake/position", Float32, self.brake_callback)

    def update(self):
        pass

    def spin(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def proximity_callback(self, msg):
        print msg

    def brake_callback(self, msg):
        print msg



if __name__ == "__main__":
    try:
        control_test = ControlTesting()
        control_test.spin()
    except rospy.ROSInterruptException:
        pass
