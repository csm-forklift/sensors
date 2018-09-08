#!/usr/bin/env python

"""Code for testing the IMU (Adafruit BNO055) on the Raspberry Pi"""

import sys
import struct
import time
import numpy as np
from array import array
import RPi.GPIO as gpio
import rospy
from encoders.msg import ImuArray, ImuMag, ImuCalibStatus, ImuCalibration
from Adafruit_BNO055 import BNO055

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

        # Set GPIO
        # FIXME: use the reset pin before starting code, this is required right
        # now since the code is not able to finish and causes a read error on 
        # the next startup
        self.reset_pin = 18
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.reset_pin, gpio.OUT)
        print "Reseting IMU"
        gpio.output(self.reset_pin, 0)
        time.sleep(0.3)
        gpio.output(self.reset_pin, 1)
        
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
        #self.serial_port = "/dev/ttyUSB0"
        print "Beginning connection"
        bno = BNO055.BNO055(serial_port=self.serial_port)
        result = bno.begin(BNO055.OPERATION_MODE_M4G)
        print "Connection: {0}".format(result)
        if not result:
            raise RuntimeError("Failed to initialize BNO055. Check sensor connection.")

        print "Board: %s" % gpio.BOARD
        print "BCM: %s" % gpio.BCM
        print "GPIO mode: %s" % gpio.getmode()
        
        # Upload calibration parameters
        calibration_values = [self.calibration.accel_offset_x, \
                              self.calibration.accel_offset_y, \
                              self.calibration.accel_offset_z, \
                              self.calibration.accel_radius, \
                              self.calibration.gyro_offset_x, \
                              self.calibration.gyro_offset_y, \
                              self.calibration.gyro_offset_z, \
                              self.calibration.mag_offset_x, \
                              self.calibration.mag_offset_y, \
                              self.calibration.mag_offset_z, \
                              self.calibration.mag_radius]
        # Random values
        #calibration_values = [1, 5, -17, 10, 25, 567, -3, 4, 9, 20, 55]
        
        # Converting calibration values into the data list required by Adafruits
        # set_calibration method
        # Convert integers into a list of 22 bytes (11 int16 values)
        int16_list = [struct.pack('h', x) for x in calibration_values]
        # combined into one string to easily divide into bytes in the next line
        bytes_joined = "".join(int16_list)
        separated = list(bytes_joined)
        # Convert hex character code bytes into values
        cal1 = [ord(x) for x in separated]
        print "cal1: "
        print cal1
        
        bno.set_calibration(cal1)
        
        print "Calibration stored"
        
        cal2 = bno.get_calibration()
        cal2_char = [struct.pack('B', x) for x in cal2]
        combined = "".join(cal2_char)
        calib_values = struct.unpack('hhhhhhhhhhh', combined)
        print calib_values



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
    finally:
        gpio.cleanup()
