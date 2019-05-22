#!/usr/bin/env python
'''
Converts a TwistStamped message into a TwistWithCovarianceStamped message.
'''

import rospy
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped

def twistCallback(msg):
    twist_covariance = TwistWithCovarianceStamped()
    twist_covariance.header = msg.header
    twist_covariance.twist.twist = msg.twist
    msg_pub.publish(twist_covariance)

if __name__ == "__main__":
    try:
        rospy.init_node("twiststamped_to_covariance")
        msg_sub = rospy.Subscriber("/twist_stamped", TwistStamped, twistCallback)
        msg_pub = rospy.Publisher("/twist_covariance", TwistWithCovarianceStamped, queue_size=5)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
