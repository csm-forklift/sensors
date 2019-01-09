#!/usr/bin/env python

import sys
import traceback
import time
import math
import rospy
from std_msgs.msg import Float32
from Phidget22.Devices.Stepper import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *

#=================================================#
# Phidget Helper Functions
#=================================================#
class EndProgramSignal(Exception):
    def __init__(self, value):
        self.value = str(value)

def DisplayError(e):
    sys.stderr.write("Desc: " + e.details + "\n")

    if (e.code == ErrorCode.EPHIDGET_WRONGDEVICE):
        sys.stderr.write("\tThis error commonly occurs when the Phidget function you are calling does not match the class of the channel that called it.\n"
                        "\tFor example, you would get this error if you called a PhidgetVoltageInput_* function with a PhidgetDigitalOutput channel.")
    elif (e.code == ErrorCode.EPHIDGET_NOTATTACHED):
        sys.stderr.write("\tThis error occurs when you call Phidget functions before a Phidget channel has been opened and attached.\n"
                        "\tTo prevent this error, ensure you are calling the function after the Phidget has been opened and the program has verified it is attached.")
    elif (e.code == ErrorCode.EPHIDGET_NOTCONFIGURED):
        sys.stderr.write("\tThis error code commonly occurs when you call an Enable-type function before all Must-Set Parameters have been set for the channel.\n"
                        "\tCheck the API page for your device to see which parameters are labled \"Must be Set\" on the right-hand side of the list.")

def PrintOpenErrorMessage(e, ph):
    sys.stderr.write("Runtime Error -> Opening Phidget Channel: \n\t")
    DisplayError(e)
    if(e.code == ErrorCode.EPHIDGET_TIMEOUT):
        sys.stderr.write("\nThis error commonly occurs if your device is not connected as specified, "
                         "or if another program is using the device, such as the Phidget Control Panel.\n")
        sys.stderr.write("\nIf your Phidget has a plug or terminal block for external power, ensure it is plugged in and powered.\n")
        if(     ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGEINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_VOLTAGERATIOINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALINPUT
            and ph.getChannelClass() != ChannelClass.PHIDCHCLASS_DIGITALOUTPUT
        ):
            sys.stderr.write("\nIf you are trying to connect to an analog sensor, you will need to use the "
                              "corresponding VoltageInput or VoltageRatioInput API with the appropriate SensorType.\n")

        if(ph.getIsRemote()):
            sys.stderr.write("\nEnsure the Phidget Network Server is enabled on the machine the Phidget is plugged into.")


class SteeringController():
    def __init__(self):
        # Set control mode
        # 0 = step mode (target position)
        # 1 = run mode (target velocity)
        self.control_mode = 0

        #=========================#
        # Create ROS Node Objects
        #=========================#
        rospy.init_node("steering_controller")
        rospy.on_shutdown(self.close) # shuts down the Phidget properly
        if (self.control_mode == 0):
            rospy.Subscriber("~target_position", Float32, self.target_position_callback, queue_size = 1)
        elif (self.control_mode == 1):
            rospy.Subscriber("~target_velocity", Float32, self.target_velocity_callback, queue_size = 1)
        else:
            print("Invalid control mode specified. Please set 'control_mode' to be 0 or 1")
            sys.exit(1)
        self.position_pub = rospy.Publisher("~current_position", Float32, queue_size = 10)
        self.velocity_pub = rospy.Publisher("~current_velocity", Float32, queue_size = 10)

        #================================#
        # Create phidget stepper channel
        #================================#
        try:
            self.ch = Stepper()
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Creating Stepper")
            raise
        except RuntimeError as e:
            sys.stderr.write("Runtime Error -> Creating Stepper")
            raise

        self.ch.setDeviceSerialNumber(522972)
        self.ch.setChannel(0)

        # Set handlers, these are called when certain Phidget events happen
        print("\n--------------------------------------")
        print("Starting up Phidget controller")
        print("* Setting OnAttachHandler...")
        self.ch.setOnAttachHandler(self.onAttachHandler)

        print("* Setting OnDetachHandler...")
        self.ch.setOnDetachHandler(self.onDetachHandler)

        print("* Setting OnErrorHandler...")
        self.ch.setOnErrorHandler(self.onErrorHandler)

        print("* Setting OnPositionChangeHandler...")
        self.ch.setOnPositionChangeHandler(self.onPositionChangeHandler)

        print("* Setting OnVelocityChangeHandler...")
        self.ch.setOnVelocityChangeHandler(self.onVelocityChangeHandler)

        # Attach to Phidget
        print("* Opening and Waiting for Attachment...")
        try:
            self.ch.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, self.ch)
            raise EndProgramSignal("Program Terminated: Open Failed")

        # Set Rescale Factor
        # (pi rad / 180 deg) * (1.8deg/step * (1/16) step) / (Gear Ratio = 26 + (103/121))
        self.ch.setRescaleFactor((math.pi/180.)*(1.8/16)/(26.+(103./121.))) # converts steps to radians

        # Set max velocity for position control
        if (self.control_mode == 0):
            speed = 60 # rpm
            speed = speed*(2*math.pi)*(1./60.) # rad/s
            self.ch.setVelocityLimit(speed)

        # Set control mode (You must either uncomment the line below or do not set the control mode at all and leave let it be the default of 0, or "step" mode. Setting self.ch.setControlMode(ControlMode.CONTROL_MODE_STEP) does not work for some reason)
        if (self.control_mode == 1):
            self.ch.setControlMode(ControlMode.CONTROL_MODE_RUN)

        #===============#
        # Run main loop
        #===============#
        # self.mainLoop()
        rospy.spin()

    def onAttachHandler(self, channel):
        ph = channel

        try:
            # Get channel info
            channelClassName = ph.getChannelClassName()
            serialNumber = ph.getDeviceSerialNumber()
            channel_num = ph.getChannel()

            # DEBUG: print channel info
            print("\nAttaching Channel")
            print("* Channel Class: " + channelClassName + "\n* Serial Number: " + str(serialNumber) + "\n* Channel: " + str(channel_num) + "\n")

            # Set data time interval
            interval_time = 32 # ms (this will publish at roughly 30Hz)
            print("Setting DataInterval to %ims" % interval_time)
            try:
                ph.setDataInterval(interval_time)
            except PhidgetException as e:
                sys.stderr.write("Runtime Error -> Setting DataInterval\n")
                return

            # Engage stepper
            print("Engaging Stepper")
            try:
                ph.setEngaged(True)
            except PhidgetException as e:
                sys.stderr.write("Runtime Error -> Setting Engaged\n")
                return

        except PhidgetException as e:
            print("Error in attach event:")
            traceback.print_exc()
            return

    def onDetachHandler(self, channel):
        ph = channel

        try:
            # Get channel info
            channelClassName = ph.getChannelClassName()
            serialNumber = ph.getDeviceSerialNumber()
            channel_num = ph.getChannel()

            # DEBUG: print channel info
            print("\nDetaching Channel")
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) + "\n\t-> Channel: " + str(channel_num) + "\n")

        except PhidgetException as e:
            print("\nError in Detach Event:")
            traceback.print_exc()
            return

    def onErrorHandler(self, channel, errorCode, errorString):
        sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")

    def onPositionChangeHandler(self, channel, position):
        self.position_pub.publish(Float32(position))

    def onVelocityChangeHandler(self, channel, velocity):
        self.velocity_pub.publish(Float32(velocity))

    def close(self):
        print("\n" + 30*"-")
        print("Closing down Phidget controller")
        self.ch.setOnPositionChangeHandler(None)
        print("Cleaning up...")
        self.ch.close()
        print("Exiting...")
        return 0

    def target_position_callback(self, msg):
        if (msg.data > self.ch.getMaxPosition()):
            targetPosition = self.ch.getMaxPosition()
            print("Desired position greater than max, setting to max value of %.2f" % self.ch.getMaxPosition())
        elif (msg.data < self.ch.getMinPosition()):
            targetPosition = self.ch.getMinPosition()
            print("Desired position less than min, setting to min value of %.2f" % self.ch.getMinPosition())
        else:
            targetPosition = msg.data

        try:
            self.ch.setTargetPosition(targetPosition)
        except PhidgetException as e:
            DisplayError(e)

    def target_velocity_callback(self, msg):
        if (msg.data > self.ch.getMaxVelocityLimit()):
            targetVelocity = self.ch.getMaxVelocityLimit()
            print("Desired velocity greater than max, setting to max value of %.2f" % self.ch.getMaxVelocityLimit())
        elif (msg.data < self.ch.getMinVelocityLimit()):
            targetVelocity = self.ch.getMinVelocityLimit()
            print("Desired velocity less than min, setting to min value of %.2f" % self.ch.getMinVelocityLimit())
        else:
            targetVelocity = msg.data

        try:
            self.ch.setVelocityLimit(targetVelocity)
        except PhidgetException as e:
            DisplayError(e)

def main():
    steering_controller = SteeringController()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
