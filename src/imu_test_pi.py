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
#        self.reset_pin = 18
#        gpio.setmode(gpio.BOARD)
#        gpio.setup(self.reset_pin, gpio.OUT)
#        gpio.output(self.reset_pin, 1)
#        time.sleep(.5)
#        gpio.output(self.reset_pin, 0)

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
        bno = BNO055.BNO055(serial_port=self.serial_port, rst=18)
        if not bno.begin(BNO055.OPERATION_MODE_NDOF):
            raise RuntimeError("Failed to initialize BNO055. Check sensor connection.")

        print "Board: %s" % gpio.BOARD
        print "BCM: %s" % gpio.BCM
        print "GPIO mode: %s" % gpio.getmode()

#        # FIXME: test calibration
#        system, gyro, accel, mag = bno.get_calibration_status()
#        try:
#            while system != 3:
#                print("sys: {0}, gyro: {1}, accel: {2}, mag: {3}".format(system, gyro, accel, mag))
#                system, gyro, accel, mag = bno.get_calibration_status()
#                time.sleep(1)
#        except:
#            pass


#        # Upload parameters to IMU
#        tmp = bno.get_calibration()
        
        #cal1 = int(cal1)
        #cal1 = [1,2,3,4,5,6,7,8,9,10,11]
        #cal1 = np.array([1,2,3,4,5,6,7,8,9,10,11], dtype='int16')
#        cal1 = []
#        for i in range(1,12):
#            cal1.append(struct.pack('h', i))
        #cal1 = array('h', [1,2,3,4,5,6,7,8,9,10,11])
#        cal1 = struct.pack('hhhhhhhhhhh',1,2,3,4,5,6,7,8,9,10,11)
#        cal1 = list(cal1)
#        cal1 = [hex(ord(x)) for x in cal1]
#        print "Cal: %s" % cal1
#        cal1 = [0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0xb1, 0x05, 0x00, 0x06, \
#                0x00, 0x07, 0x00, 0x08, 0x00, 0xbb, 0x00, 0x0a, 0x00, 0x0b, 0x00]
#        cal1 = [1, 0, 5, 0, 239, 255, 10, 0, 25, 0, 55, \
#                2, 253, 255, 4, 0, 9, 0, 20, 0, 11, 0]
        
        calibration_values = [1, 5, -17, 10, 25, 567, -3, 4, 9, 20, 55]
        
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
    except KeyboardInterrupt:
        gpio.cleanup()
