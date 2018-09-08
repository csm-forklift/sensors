#!/usr/bin/env python

"""Code for testing the IMU (Adafruit BNO055) on the Raspberry Pi"""

# IMPORTANT!
# The calibration bytes need to be placed in this order:
# accel_offset_x
# accel_offset_y
# accel_offset_z
# mag_offset_x
# mag_offset_y
# mag_offset_z
# gyro_offset_x
# gyro_offset_y
# gyro_offset_z
# accel_radius
# mag_radius

import sys
import struct
import time # for sleep, time
import numpy as np
from array import array
import RPi.GPIO as gpio
import rospy
from sensors.msg import ImuArray, ImuMag, ImuCalibStatus, ImuCalibration
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

class Quat():
    def __init__(self, qx=0.0, qy=0.0, qz=0.0, qw=0.0):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        
class Vec():
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

def bytes_from_calibration(calibration):
        '''This function converts the calibration parameters into a list of 
        bytes as required by Adafruits set_calibration() method.'''
        calibration_list = [calibration.accel_offset_x, \
                            calibration.accel_offset_y, \
                            calibration.accel_offset_z, \
                            calibration.mag_offset_x, \
                            calibration.mag_offset_y, \
                            calibration.mag_offset_z, \
                            calibration.gyro_offset_x, \
                            calibration.gyro_offset_y, \
                            calibration.gyro_offset_z, \
                            calibration.accel_radius, \
                            calibration.mag_radius]
        # Convert integers into a list of 22 bytes (11 int16 values in Hex form)
        int16_list = [struct.pack('h', x) for x in calibration_list]
        # combine into one string to easily divide into bytes in the next line
        bytes_joined = "".join(int16_list)
        byte_list = list(bytes_joined)
        # Convert hex character code bytes into values
        cal_bytes = [ord(x) for x in byte_list]
        return cal_bytes

def calibration_from_bytes(cal_bytes):
        '''This function converts a list of int16 bytes back into a list of 
        integer values'''
        # Convert numbers into unsigned byte Hex representation
        byte_list = [struct.pack('B', x) for x in cal_bytes]
        # Join into a single string for easily unpacking in the next line
        bytes_joined = "".join(byte_list)
        # Convert Hex string of 22 bytes into 11 integers
        calibration_list = struct.unpack('hhhhhhhhhhh', bytes_joined)
        calibration = Calibration()
        calibration.accel_offset_x = calibration_list[0]
        calibration.accel_offset_y = calibration_list[1]
        calibration.accel_offset_z = calibration_list[2]
        calibration.mag_offset_x = calibration_list[3]
        calibration.mag_offset_y = calibration_list[4]
        calibration.mag_offset_z = calibration_list[5]
        calibration.gyro_offset_x = calibration_list[6]
        calibration.gyro_offset_y = calibration_list[7]
        calibration.gyro_offset_z = calibration_list[8]
        calibration.accel_radius = calibration_list[9]
        calibration.mag_radius = calibration_list[10]
        return calibration

class ImuTest():
    def __init__(self):
        #===== Setup ROS node, publishers, and messages =====#
        rospy.init_node("imu_test_pi")
        self.imu_data = ImuArray()
        self.imu_pub = rospy.Publisher("/imu/data_array", ImuArray, queue_size=1)
        self.imu_mag = ImuMag()
        self.mag_pub = rospy.Publisher("/imu/mag_array", ImuMag, queue_size=1)
        self.imu_status = ImuCalibStatus()
        self.status_pub = rospy.Publisher("/imu/status", ImuCalibStatus, queue_size=1)
        self.imu_calib = ImuCalibration()
        self.calib_pub = rospy.Publisher("/imu/calibration", ImuCalibration, queue_size=1)
        self.rate = 30 # publish at 30 Hz
        self.save_time = time.time()

        #===== Set GPIO =====#
        gpio.setmode(gpio.BOARD)
        # reseting the board causes problems, so don't
        
        #======================================================================#
        # IMU Initialization
        #======================================================================#
        #===== Set calibration defaults
        self.calibration = Calibration()

        #===== Read in configuration parameters
        calibration_param_names = ["accel_offset_x", "accel_offset_y", "accel_offset_z", "accel_radius", \
            "gyro_offset_x", "gyro_offset_y", "gyro_offset_z", \
            "mag_offset_x", "mag_offset_y", "mag_offset_z", "mag_radius"]
        for param in calibration_param_names:
            if rospy.has_param("imu/calibration/" + param):
                exec "self.calibration." + param + " = rospy.get_param('imu/calibration/" + param + "')"
            else:
                print "No '" + param + "' found, using {0}".format(eval("self.calibration." + param))
                
        #===== Begin BNO055
        self.serial_port = "/dev/serial0"
        self.bno = BNO055.BNO055(serial_port=self.serial_port)

        # Initial mode should be OPERATION_MODE_M4G so magnetometer can align 
        # without calibration, then after loading calibration change mode to
        # OPERATION_MODE_NDOF
        if not self.bno.begin(BNO055.OPERATION_MODE_M4G):
            raise RuntimeError("Failed to initialize BNO055. Check sensor connection.")
        
        #===== Upload calibration parameters
        # Convert calibration to bytes
        initial_cal = bytes_from_calibration(self.calibration)
        
        # Upload to IMU
        self.bno.set_calibration(initial_cal)
        
        # Change mode to OPERATION_MODE_NDOF so sensor data is fused to give
        # absolute orientation
        self.bno.set_mode(BNO055.OPERATION_MODE_NDOF)
        
        rospy.loginfo("IMU initialization successful.")
        #======================================================================#
        # IMU Initialization
        #======================================================================#

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):
        #===== Read IMU data
        quat = Quat()
        ang = Vec()
        lin = Vec()
        mag = Vec()
        quat.qx, quat.qy, quat.qz, quat.qw = self.bno.read_quaternion()
        ang.x, ang.y, ang.z = self.bno.read_gyroscope()
        lin.x, lin.y, lin.z = self.bno.read_accelerometer()
        mag.x, mag.y, mag.z = self.bno.read_magnetometer()
        
        #===== Convert to ROS message
        # IMU data
        self.imu_data.data[0] = quat.qw
        self.imu_data.data[1] = quat.qx
        self.imu_data.data[2] = quat.qy
        self.imu_data.data[3] = quat.qz
        self.imu_data.data[4] = ang.x
        self.imu_data.data[5] = ang.y
        self.imu_data.data[6] = ang.z
        self.imu_data.data[7] = lin.x
        self.imu_data.data[8] = lin.y
        self.imu_data.data[9] = lin.z
        # Magnetometer data
        self.imu_mag.data[0] = mag.x
        self.imu_mag.data[1] = mag.y
        self.imu_mag.data[2] = mag.z
        # Calibration Status
        self.imu_status.system, self.imu_status.gyro, self.imu_status.accel, self.imu_status.mag = self.bno.get_calibration_status()
        # Calibration parameters
        # (if system status is 3 and last save time is >60sec)
        if ((self.imu_status == 3) and (time.time() - self.save_time) > 60.0):
            self.calibration = bytes_from_calibration(self.bno.get_calibration())
            
            self.imu_calib.data[0] = self.calibration.accel_offset_x
            self.imu_calib.data[1] = self.calibration.accel_offset_y
            self.imu_calib.data[2] = self.calibration.accel_offset_z
            self.imu_calib.data[3] = self.calibration.accel_radius
            self.imu_calib.data[4] = self.calibration.gyro_offset_x
            self.imu_calib.data[5] = self.calibration.gyro_offset_y
            self.imu_calib.data[6] = self.calibration.gyro_offset_z
            self.imu_calib.data[7] = self.calibration.mag_offset_x
            self.imu_calib.data[8] = self.calibration.mag_offset_y
            self.imu_calib.data[9] = self.calibration.mag_offset_z
            self.imu_calib.data[10] = self.calibration.mag_radius
            
            self.save_time = time.time()
            self.calib_pub.publish(self.imu_calib)

        #===== Publish
        self.imu_pub.publish(self.imu_data)
        self.mag_pub.publish(self.imu_mag)
        self.status_pub.publish(self.imu_status)

if __name__ == "__main__":
    try:
        imu = ImuTest()
        imu.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        gpio.cleanup()
