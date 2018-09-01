#!/usr/bin/env python

''' This node receives data published from an Arduino as arrays and then
converts the data into ROS IMU messages and republishes. This node also handles
checking the calibration status of the IMU and stores those parameters in a yaml
file if the status is all 3's (meaning, fully calibrated).'''

import rospy
import rospkg
from sensor_msgs.msg import Imu
from encoders.msg import ImuArray
from encoders.msg import ImuMag
from encoders.msg import ImuCalibration
from encoders.msg import ImuCalibStatus
from geometry_msgs.msg import Vector3Stamped
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
from tf.transformations import euler_from_quaternion
from time import time, localtime

class ImuRead():
    def __init__(self):
        rospy.init_node("imu_read");

        # Subscribers and frames for Arduino data
        rospy.Subscriber("imu/data_array", ImuArray, self.imu_array_callback, queue_size = 1)
        rospy.Subscriber("imu/mag_array", ImuMag, self.imu_mag_callback, queue_size = 1)
        rospy.Subscriber("imu/calibration", ImuCalibration, self.imu_calib_callback, queue_size = 1)
        self.odom_frame_id = "odom"
        self.base_frame_id = "base_link"
        self.imu_frame_id = "imu_link"

        # Messages and publishers for conversion
        self.imu_msg = Imu()
        self.imu_base_link_msg = Imu()
        self.imu_msg.header.frame_id = self.imu_frame_id
        self.imu_base_link_msg.header.frame_id = self.base_frame_id
        self.imu_pub = rospy.Publisher("imu/data", Imu, queue_size = 10)
        self.imu_rpy_msg = Vector3Stamped()
        self.imu_rpy_pub = rospy.Publisher("imu/rpy", Vector3Stamped, queue_size = 10)
        self.imu_mag_msg = Vector3Stamped()
        self.imu_mag_pub = rospy.Publisher("imu/mag", Vector3Stamped, queue_size = 10)
        # Add transform to visualize orientation
        #self.odom_broadcaster = TransformBroadcaster()
        # Listener for base_link to imu_link transform to convert imu message into base_link frame

        # Parameters for checking and storing calibration data
        self.last_save_time = time()
        self.calibration = ImuCalibration()
        # Obtain the filepath for the IMU package (currently in 'encoders')
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('encoders')
        self.imu_calib_filename = rospy.get_param("/imu/calib_filename", self.package_path + "/config/imu_calibration.yaml")

        # Procedure
        # -save initial time of startup for when the calibration parameters were set
        # -save calibration status and values when messages are received
        # -during update function, check whether the calibration status is all 3's
        # -if the status is all 3's then get the time since that last saved file
        # -if time since last save has been one minute or more, save the new
        #   parameters in a yaml file and update the last save time

        # Loop rate for repeated code
        self.rate = rospy.Rate(30)

    def spin(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

    def update(self):
        pass

    def imu_array_callback(self, msg):
        # Message is simply an array with 10 elements
        # 0-3 = quaternion w,x,y,z
        # 4-6 = angular velocity x,y,z
        # 7-9 = linear acceleration x,y,z
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.orientation.w = msg.data[0]
        self.imu_msg.orientation.x = msg.data[1]
        self.imu_msg.orientation.y = msg.data[2]
        self.imu_msg.orientation.z = msg.data[3]
        self.imu_msg.angular_velocity.x = msg.data[4]
        self.imu_msg.angular_velocity.y = msg.data[5]
        self.imu_msg.angular_velocity.z = msg.data[6]
        self.imu_msg.linear_acceleration.x = msg.data[7]
        self.imu_msg.linear_acceleration.y = msg.data[8]
        self.imu_msg.linear_acceleration.z = msg.data[9]

        self.imu_pub.publish(self.imu_msg)

        # Convert quaternion to RPY angles for /imu/rpy topic
        self.imu_rpy_msg.header = self.imu_msg.header
        (r, p, y) = euler_from_quaternion([self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w])
        self.imu_rpy_msg.vector.x = r
        self.imu_rpy_msg.vector.y = p
        self.imu_rpy_msg.vector.z = y

        self.imu_rpy_pub.publish(self.imu_rpy_msg)

        # self.odom_broadcaster.sendTransform(
        #     (0, 0, 0),
        #     (self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w),
        #     rospy.Time.now(),
        #     self.base_frame_id,
        #     self.odom_frame_id
        # )

    def imu_mag_callback(self, msg):
        # Convert array to Vector3Stamped message
        self.imu_mag_msg.header = self.imu_msg.header
        self.imu_mag_msg.header.stamp = rospy.Time.now()
        self.imu_mag_msg.vector.x = msg.data[0]
        self.imu_mag_msg.vector.y = msg.data[1]
        self.imu_mag_msg.vector.z = msg.data[2]

        self.imu_mag_pub.publish(self.imu_mag_msg)

    def imu_calib_callback(self, msg):
        self.calibration = msg
        # Get current time
        self.last_save_time = time()
        time_struct = localtime(self.last_save_time)
        # Write configuration parameters to yaml file
        with open(self.imu_calib_filename, "w") as file:
            file.write("# Calibration file generated at %s:%s:%s on %s-%s-%s\n" % (time_struct.tm_hour, time_struct.tm_min, time_struct.tm_sec, time_struct.tm_year, time_struct.tm_mon, time_struct.tm_mday))
            file.write("# These values are loaded with the startup launch file and uploaded to the IMU from the microcontroller.\n")
            file.write("accel_offset_x: %d\n" % (self.calibration.data[0]))
            file.write("accel_offset_y: %d\n" % (self.calibration.data[1]))
            file.write("accel_offset_z: %d\n" % (self.calibration.data[2]))
            file.write("accel_radius: %d\n" % (self.calibration.data[3]))
            file.write("gyro_offset_x: %d\n" % (self.calibration.data[4]))
            file.write("gyro_offset_y: %d\n" % (self.calibration.data[5]))
            file.write("gyro_offset_z: %d\n" % (self.calibration.data[6]))
            file.write("mag_offset_x: %d\n" % (self.calibration.data[7]))
            file.write("mag_offset_y: %d\n" % (self.calibration.data[8]))
            file.write("mag_offset_z: %d\n" % (self.calibration.data[9]))
            file.write("mag_radius: %d\n" % (self.calibration.data[10]))
            file.close();

        rospy.loginfo("New IMU configuration parameters written to file: %s" % (self.imu_calib_filename))


if __name__ == "__main__":
    try:
        imu_read = ImuRead()
        imu_read.spin()
    except rospy.ROSInterruptException:
        pass
