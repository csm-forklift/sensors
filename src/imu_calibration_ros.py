#!/usr/bin/env python

'''
Reads IMU calibration data published from an Arduino with
'imu_calibration_ros.ino'. Then stores it to a '.yaml' file.
'''


import rospy
import rospkg
from sensors.msg import ImuCalibStatus, ImuCalibration
import time
import datetime

class CalibrationSave:
    def __init__(self):
        # Calibration Data
        self.calibration = ImuCalibration()
        self.calibration.data[0] = None # set to None until the first message is received

        # Setup ROS Objects
        rospy.init_node("imu_calibration_ros")
        self.status_sub = rospy.Subscriber("/imu/status", ImuCalibStatus, self.status_callback, queue_size=3)
        self.calib_sub = rospy.Subscriber("/imu/calibration", ImuCalibration, self.calibration_callback, queue_size=3)

        # Get file path for package "sensors"
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path('sensors')
        self.file_path += '/config/'

        rospy.spin()

    def status_callback(self, msg):
        if self.calibration.data[0] is not None:
            # Check if status is all 3s
            if (msg.system == 3 and msg.accel == 3 and msg.gyro == 3 and msg.mag == 3):
                # Set time stamp and create file
                timestamp = datetime.datetime.fromtimestamp(time.time()).strftime("%Y-%m-%d_%H%M%S")
                file = open(self.file_path + "imu_calibration-" + timestamp + ".yaml", 'w')

                # Print file header content
                header = "# Calibration file generated at " + timestamp + "\n# These values are loaded with the startup launch file and uploaded to the IMU\n# from the microcontroller.\n"
                file.write(header)

                # Write calibration data
                file.write("accel_offset_x: {:d}\n".format(self.calibration.data[0]))
                file.write("accel_offset_y: {:d}\n".format(self.calibration.data[1]))
                file.write("accel_offset_z: {:d}\n".format(self.calibration.data[2]))
                file.write("accel_radius: {:d}\n".format(self.calibration.data[3]))
                file.write("gyro_offset_x: {:d}\n".format(self.calibration.data[4]))
                file.write("gyro_offset_y: {:d}\n".format(self.calibration.data[5]))
                file.write("gyro_offset_z: {:d}\n".format(self.calibration.data[6]))
                file.write("mag_offset_x: {:d}\n".format(self.calibration.data[7]))
                file.write("mag_offset_y: {:d}\n".format(self.calibration.data[8]))
                file.write("mag_offset_z: {:d}\n".format(self.calibration.data[9]))
                file.write("mag_radius: {:d}\n".format(self.calibration.data[10]))

                # Close file
                file.close()

                # Close the node
                rospy.loginfo("Calibration complete. Closing Node...")
                rospy.signal_shutdown("calibration recorded, closing node")

    def calibration_callback(self, msg):
        self.calibration.data[0] = msg.data[0]
        self.calibration.data[1] = msg.data[1]
        self.calibration.data[2] = msg.data[2]
        self.calibration.data[3] = msg.data[3]
        self.calibration.data[4] = msg.data[4]
        self.calibration.data[5] = msg.data[5]
        self.calibration.data[6] = msg.data[6]
        self.calibration.data[7] = msg.data[7]
        self.calibration.data[8] = msg.data[8]
        self.calibration.data[9] = msg.data[9]
        self.calibration.data[10] = msg.data[10]

if __name__ == "__main__":
    try:
        calibration_save = CalibrationSave()
    except rospy.ROSInterruptException:
        pass
