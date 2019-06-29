#!/usr/bin/env python
'''
Calculates the covariance for the 'twist' and 'imu' messages.
'''

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
import math

class CalculateCovariance:
    def __init__(self):
        # Parameters
        self.lin_vel_std = 0.1 # linear velocity standard deviation
        self.angle_std = 1.38*(math.pi/180.) # angle standard deviation
        # Calculated from data sheet
        self.accel_std = 0.01470998
        self.gyro_std = 0.3

        #===== Test Standard Deviation Values =====#
        self.angle_std = 0
        self.lin_vel_std = 0
        self.accel_std = 0
        self.gyro_std = 0
        self.orient_std = 0
        #==========================================#

        self.forklift_body_length = 2.5601
        self.velocity = 0
        self.angle = 0

        # Initialize ROS Objects
        rospy.init_node("calculate_covariance")

        self.vel_sub = rospy.Subscriber("/velocity_node/velocity", Float64, self.vel_callback, queue_size=3)
        self.angle_sub = rospy.Subscriber("/steering_node/filtered_angle", Float64, self.angle_callback, queue_size=3)
        self.twist_sub = rospy.Subscriber("/velocity_conversion/twist_in", TwistWithCovarianceStamped, self.twist_callback, queue_size=3)
        self.imu0_sub = rospy.Subscriber("/arduino/imu0_in", Imu, self.imu0_callback, queue_size=3)
        self.imu1_sub = rospy.Subscriber("/arduino/imu1_in", Imu, self.imu1_callback, queue_size=3)

        self.twist_pub = rospy.Publisher("~twist", TwistWithCovarianceStamped, queue_size=1)
        self.imu0_pub = rospy.Publisher("~imu0", Imu, queue_size=1)
        self.imu1_pub = rospy.Publisher("~imu1", Imu, queue_size=1)

        rospy.spin()

    def vel_callback(self, msg):
        self.velocity = msg.data

    def angle_callback(self, msg):
        self.angle = msg.data

    def twist_callback(self, msg):
        # Calculate covariange
        dtheta_dv = math.tan(self.angle)/self.forklift_body_length
        dtheta_ddelta = (self.velocity/self.forklift_body_length)*(1/math.cos(self.angle))**2
        theta_dot_std = math.sqrt((dtheta_dv*self.lin_vel_std)**2 + (dtheta_ddelta*self.angle_std)**2)

        msg.twist.covariance = [self.lin_vel_std**2, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, theta_dot_std**2]

        self.twist_pub.publish(msg)

    def imu0_callback(self, msg):
        # Convert Gyro data from Degrees Per Second to Radians Per Second
        # (only required if using data older than 2019-06-26)
        msg.angular_velocity.x *= (math.pi/180.0)/8
        msg.angular_velocity.y *= (math.pi/180.0)/8
        msg.angular_velocity.z *= (math.pi/180.0)/8

        msg.orientation_covariance = [self.orient_std**2, 0, 0,
                                      0, self.orient_std**2, 0,
                                      0, 0, self.orient_std**2]
        msg.angular_velocity_covariance = [self.gyro_std**2, 0, 0,
                                           0, self.gyro_std**2, 0,
                                           0, 0, self.gyro_std**2]
        msg.linear_acceleration_covariance = [self.accel_std**2, 0, 0,
                                              0, self.accel_std**2, 0,
                                              0, 0, self.accel_std**2]
        self.imu0_pub.publish(msg)

    def imu1_callback(self, msg):
        # Convert Gyro data from Degrees Per Second to Radians Per Second
        # (only required if using data older than 2019-06-26)
        msg.angular_velocity.x *= (math.pi/180.0)/8
        msg.angular_velocity.y *= (math.pi/180.0)/8
        msg.angular_velocity.z *= (math.pi/180.0)/8

        msg.orientation_covariance = [self.orient_std**2, 0, 0,
                                      0, self.orient_std**2, 0,
                                      0, 0, self.orient_std**2]
        msg.angular_velocity_covariance = [self.gyro_std**2, 0, 0,
                                           0, self.gyro_std**2, 0,
                                           0, 0, self.gyro_std**2]
        msg.linear_acceleration_covariance = [self.accel_std**2, 0, 0,
                                              0, self.accel_std**2, 0,
                                              0, 0, self.accel_std**2]
        self.imu1_pub.publish(msg)

if __name__ == "__main__":
    try:
        calculate_covariance = CalculateCovariance()
    except rospy.ROSInterruptException:
        pass
