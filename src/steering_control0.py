#!/usr/bin/env python

import sys
import traceback
import time
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
        # Create phidget stepper channel
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

        # Set handlers
        print("\n--------------------------------------")
        print("\nSetting OnAttachHandler...")
        self.ch.setOnAttachHandler(self.onAttachHandler)

        print("Setting OnDetachHandler...")
        self.ch.setOnDetachHandler(self.onDetachHandler)

        print("Setting OnErrorHandler...")
        self.ch.setOnErrorHandler(self.onErrorHandler)

        print("\nSetting OnPositionChangeHandler...")
        self.ch.setOnPositionChangeHandler(self.onPositionChangeHandler)

        print("\nOpening and Waiting for Attachment...")
        try:
            self.ch.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, self.ch)
            raise EndProgramSignal("Program Terminated: Open Failed")

        # Set Rescale Factor
        # 1.8deg/step * (1/16) step / (Gear Ratio = 26 + (103/121))
        self.ch.setRescaleFactor((1.8/16)/(26.+(103./121.)))

        # Set control mode
        # self.ch.setControlMode(ControlMode.CONTROL_MODE_STEP)
        #self.ch.setControlMode(ControlMode.CONTROL_MODE_RUN)

        #===== Run main loop
        self.mainLoop()

    def onAttachHandler(self, channel):
        ph = channel

        try:
            # Get channel info
            channelClassName = ph.getChannelClassName()
            serialNumber = ph.getDeviceSerialNumber()
            channel_num = ph.getChannel()

            # DEBUG: print channel info
            print("\nAttaching Channel")
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) + "\n\t-> Channel: " + str(channel_num) + "\n")

            # Set data time interval
            interval_time = 500 # ms
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

    def onPositionChangeHandler(self, channel, Position):
        print("[Position Event] -> Position: " + str(Position))

    def mainLoop(self):
        end = False
        while (end != True):
            buf = sys.stdin.readline(100)
            if not buf:
                continue

            if (buf[0] == 'Q' or buf[0] == 'q'):
                end = True
                continue

            try:
                targetPosition = float(buf)
            except ValueError as e:
                print("Input must be a number, or Q to quit.")
                continue

            if (targetPosition > self.ch.getMaxPosition() or targetPosition < self.ch.getMinPosition()):
                print("TargetPosition must be between %.2f and %.2f\n" % (self.ch.getMinPosition(), self.ch.getMaxPosition()))
                continue

            print("Setting Stepper TargetPosition to " + str(targetPosition))
            try:
                self.ch.setTargetPosition(targetPosition)
            except PhidgetException as e:
                DisplayError(e)

            print(self.ch.getTargetPosition())
            print(self.ch.getVelocity())

        self.close()

    def close(self):
        self.ch.setOnPositionChangeHandler(None)
        print("Cleaning up...")
        self.ch.close()
        print("\nExiting...")
        return 0

def main():
    steering_controller = SteeringController()
    # global stepper
    #
    # end = False
    # while (end != True):
    #     buf = sys.stdin.readline(100)
    #     if not buf:
    #         continue
    #
    #     if (buf[0] == 'Q' or buf[0] == 'q'):
    #         end = True
    #         continue
    #
    #     try:
    #         targetPosition = float(buf)
    #     except ValueError as e:
    #         print("Input must be a number, or Q to quit.")
    #         continue
    #
    #     if (targetPosition > stepper.getMaxPosition() or targetPosition < stepper.getMinPosition()):
    #         print("TargetPosition must be between %.2f and %.2f\n" % (stepper.getMinPosition(), stepper.getMaxPosition()))
    #         continue
    #
    #     print("Setting Stepper TargetPosition to " + str(targetPosition))
    #     stepper.setTargetPosition(targetPosition)
    #
    # #clear the PositionChange event handler
    # stepper.setOnPositionChangeHandler(None)
    #
    # print("Cleaning up...")
    # stepper.close()
    # print("\nExiting...")
    # return 0

main()
