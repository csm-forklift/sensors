#!/bin/bash

# Bind the USBCAN device

echo "Starting CANUSB connection on 'slcand'"
/usr/bin/logger "Starting CANUSB connection on 'slcand'"

# Get first argument and store it as device name
# if empty, use default
if [ $# -eq 0 ]
then
    DEVICE_NAME="ttyUSB0"
else
    DEVICE_NAME=$1
fi  

# -s4 argument sets rate to 125K bps
slcand -o -c -f -s4 /dev/$DEVICE_NAME slcan0 # this version takes the first argument to the script
# slcand -o -c -f -s4 /dev/ttyUSB0 slcan0 # this version lets you manually set the device location

echo "Sleep for 2 seconds"
/usr/bin/logger "Sleep for 2 seconds"

sleep 2

echo "Set to 'up' state in ifconfig"
/usr/bin/logger "Set to 'up' state in ifconfig"

ifconfig slcan0 up

echo "FINISHED ADDING CAN DEVICE"
/usr/bin/logger "FINISHED ADDING CAN DEVICE"
