#!/usr/bin/env python

"""Code for testing the IMU (Adafruit BNO055) on the Raspberry Pi"""

#import RPi.GPIO as gpio
import rospy
from encoders.msg import ImuArray, ImuMag, ImuCalibStatus, ImuCalibration
from Adafruit_BNO055 import BNO055, OPERATION_MODE_M4G, OPERATION_MODE_NDOF, OPERATION_MODE_CONFIG

class Calibration():
    def __init__(self, ax=0.0, ay=0.0, az=0.0, ar=0.0, gx=0.0, gy=0.0, \
                 gz=0.0, mx=0.0, my=0.0, mz=0.0, mr=0.0):
        self.accel_offset_x = ax
        self.accel_offset_y = ay
        self.accel_offset_z = az
        self.accel_radius = ar
        self.gyro_offset_x = gx
        self.gyro_offset_y = gy
        self.gyro_offset_z = gz
        self.mag_offset_x = mx
        self.mag_offset_y = my
        self.mag_offset_z = mz
        self.mag_radius = mr

class ImuTest():
    def __init__(self):
        # Setup ROS node, publishers, and messages
        rospy.init_node("imu_test_pi")
        self.imu_pub = rospy.Publisher("/imu/data_array", ImuArray, queue_size=1)
        self.mag_pub = rospy.Publisher("/imu/mag_array", ImuMag, queue_size=1)
        self.status_pub = rospy.Publisher("/imu/status", ImuCalibStatus, queue_size=1)
        self.calib_pub = rospy.Publisher("/imu/calibration", ImuCalibration, queue_size=1)
        self.rate = 30 # publish at 30 Hz

        # Set calibration defaults
        self.calibration = Calibration()

        # Read in configuration parameters
        calibration_param_names = ["accel_offset_x", "accel_offset_y", "accel_offset_z", "accel_radius", \
            "gyro_offset_x", "gyro_offset_y", "gyro_offset_z", \
            "mag_offset_x", "mag_offset_y", "mag_offset_z", "mag_radius"]
        for param in calibration_param_names:
            if rospy.has_param("imu/calibration/" + param):
                eval("self.calibration." + param + " = rospy.get_param('imu/calibration/" + param + "')")
            else:
                print "No '" + param + "' found, using {0}".format(eval("self.calibration." + param))

        # Begin BNO055
        self.serial_port = "/dev/serial0"
        bno = BNO055.BNO055(serial_port=self.serial_port)
        if not bno.begin(OPERATION_MODE_M4G):
            raise RuntimeError("Failed to initialize BNO055. Check sensor connection.")

        

        # Upload parameters to IMU



    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):
        # Read IMU data

        # Convert to ROS message

        # Publish
        pass

if __name__ == "__main__":
    try:
        imu = ImuTest()
        imu.spin()
    except rospy.ROSInterruptException:
        pass
