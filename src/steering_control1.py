#!/usr/bin/env python

import sys
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

'''
* Configures the device's DataInterval
* Displays info about the attached Phidget channel.
* Fired when a Phidget channel with onAttachHandler registered attaches
*
* @param self The Phidget channel that fired the attach event
'''
def onAttachHandler(self):

    ph = self

    try:
        #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
        #www.phidgets.com/docs/Using_Multiple_Phidgets for information

        print("\nAttach Event:")

        """
        * Get device information and display it.
        """
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) + "\n\t-> Channel:  " + str(channel) + "\n")

        """
        * Set the DataInterval inside of the attach handler to initialize the device with this value.
        * DataInterval defines the minimum time between PositionChange events.
        * DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
        """
        interval_time = 500 # ms
        print("\tSetting DataInterval to %ims" % interval_time)
        try:
            ph.setDataInterval(interval_time)
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Setting DataInterval: \n\t")
            DisplayError(e)
            return

        """
        * Engage the Stepper inside of the attach handler to allow the motor to move to its target position
        * The motor will only track a target position if it is engaged.
        * Engaged can be set to True to enable the servo, or False to disable it.
        """
        print("\tEngaging Stepper")
        try:
            ph.setEngaged(True)
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Setting Engaged: \n\t")
            DisplayError(e)
            return

    except PhidgetException as e:
        print("\nError in Attach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

"""
* Displays info about the detached Phidget channel.
* Fired when a Phidget channel with onDetachHandler registered detaches
*
* @param self The Phidget channel that fired the attach event
"""
def onDetachHandler(self):

    ph = self

    try:
        #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
        #www.phidgets.com/docs/Using_Multiple_Phidgets for information

        print("\nDetach Event:")

        """
        * Get device information and display it.
        """
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) + "\n\t-> Channel:  " + str(channel) + "\n")

    except PhidgetException as e:
        print("\nError in Detach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

"""
* Writes Phidget error info to stderr.
* Fired when a Phidget channel with onErrorHandler registered encounters an error in the library
*
* @param self The Phidget channel that fired the attach event
* @param errorCode the code associated with the error of enum type ph.ErrorEventCode
* @param errorString string containing the description of the error fired
"""
def onErrorHandler(self, errorCode, errorString):

    sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")

"""
* Outputs the Stepper's most recently reported Position.
* Fired when a Stepper channel with onPositionChangeHandler registered meets DataInterval criteria
*
* @param self The Stepper channel that fired the PositionChange event
* @param Position The reported Position from the Stepper channel
"""
def onPositionChangeHandler(self, Position):

    #If you are unsure how to use more than one Phidget channel with this event, we recommend going to: www.phidgets.com/docs/Using_Multiple_Phidgets for information
    print("[Position Event] -> Position: " + str(Position))

"""
* Creates, configures, and opens a Stepper channel.
* Provides interface for controlling TargetPosition of the Stepper.
* Closes out Stepper channel
*
* @return 0 if the program exits successfully, 1 if it exits with errors.
"""
def main():

    try:
        """
        * Allocate a new Phidget Channel object
        """
        try:
            ch = Stepper()
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Creating Stepper: \n\t")
            DisplayError(e)
            raise
        except RuntimeError as e:
            sys.stderr.write("Runtime Error -> Creating Stepper: \n\t" + e)
            raise

        """
        * Set matching parameters to specify which channel to open
        """
        ch.setDeviceSerialNumber(522972)
        ch.setChannel(0)

        """
        * Add event handlers before calling open so that no events are missed.
        """
        print("\n--------------------------------------")
        print("\nSetting OnAttachHandler...")
        ch.setOnAttachHandler(onAttachHandler)

        print("Setting OnDetachHandler...")
        ch.setOnDetachHandler(onDetachHandler)

        print("Setting OnErrorHandler...")
        ch.setOnErrorHandler(onErrorHandler)

        print("\nSetting OnPositionChangeHandler...")
        ch.setOnPositionChangeHandler(onPositionChangeHandler)

        """
        * Open the channel with a timeout
        """
        print("\nOpening and Waiting for Attachment...")

        try:
            ch.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, ch)
            raise EndProgramSignal("Program Terminated: Open Failed")

        print("--------------------\n"
        "\n  | Stepper motor position can be controlled by setting its Target Position.\n"
        "  | The target position can be a number between MinPosition and MaxPosition.\n"
        "  | For this example, acceleration and velocity limit are left as their default values, but can be changed in custom code.\n"

        "\nInput a desired position and press ENTER\n"
        "Input Q and press ENTER to quit\n")

        end = False

        # Rescale Factor
        # 1.8deg/step * (1/16) step / (Gear Ratio = 26 + (103/121))
        # ch.setRescaleFactor((1.8/16)/(26.+(103./121.)))
        ch.setControlMode(ControlMode.CONTROL_MODE_STEP)
        # ch.setControlMode(ControlMode.CONTROL_MODE_RUN)

        while (end != True):
            buf = sys.stdin.readline(100)
            if not buf:
                continue

            if (buf[0] == 'Q' or buf[0] ==  'q'):
                end = True
                continue

            # Position Control
            try:
                targetPosition = float(buf)
            except ValueError as e:
                print("Input must be a number, or Q to quit.")
                continue

            if (targetPosition > ch.getMaxPosition() or targetPosition < ch.getMinPosition()):
                print("TargetPosition must be between %.2f and %.2f\n" % (ch.getMinPosition(), ch.getMaxPosition()))
                continue

            print("Setting Stepper TargetPosition to " + str(targetPosition))
            ch.setTargetPosition(targetPosition)

            # # Velocity Control
            # try:
            #     targetVelocity = float(buf)
            # except ValueError as e:
            #     print("Input must be a number, or Q to quit.")
            #     continue
            #
            # if (targetVelocity > ch.getMaxVelocityLimit() or targetVelocity < ch.getMinVelocityLimit()):
            #     print("TargetPosition must be between %.2f and %.2f\n" % (ch.getMinVelocityLimit(), ch.getMaxVelocityLimit()))
            #     continue
            #
            # print("Setting Stepper VelocityLimit to " + str(targetVelocity))
            # ch.setVelocityLimit(targetVelocity)

        '''
        * Perform clean up and exit
        '''

        #clear the PositionChange event handler
        ch.setOnPositionChangeHandler(None)

        print("Cleaning up...")
        ch.close()
        print("\nExiting...")
        return 0

    except PhidgetException as e:
        sys.stderr.write("\nExiting with error(s)...")
        DisplayError(e)
        traceback.print_exc()
        print("Cleaning up...")
        #clear the PositionChange event handler
        ch.setOnPositionChangeHandler(None)
        ch.close()
        return 1
    except EndProgramSignal as e:
        print(e)
        print("Cleaning up...")
        #clear the PositionChange event handler
        ch.setOnPositionChangeHandler(None)
        ch.close()
        return 1
    finally:
        print("Press ENTER to end program.")
        readin = sys.stdin.readline()

main()
