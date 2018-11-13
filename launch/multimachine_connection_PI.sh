#!/usr/bin/env bash

# Raspberry Pi Version

# This script sets up the ROS environment variables needed to allow the Raspberry
# Pi and NUC to connect together. You must set the ROS master URI as the same
# value on each computer that needs to be connected. Then for any computer that
# will be publishing values must export the ROS_IP variable to be their IP address.

export ROS_MASTER_URI=http://forkPC.local:11311
export ROS_IP=192.168.1.2
