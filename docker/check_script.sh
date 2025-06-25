#!/bin/bash

# Source the ROS 2 setup script
source /opt/ros/humble/setup.bash
if [ -f /home/drims/drims_ws/install/setup.bash ]; then
    source /home/drims/drims_ws/install/setup.bash
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if /drims_ws exists
if [ -d /home/drims/drims_ws/src ]; then
    echo "drims ws exists"
else
    echo "drims ws does not exist"
fi

# Check if ROS 2 is installed
if command_exists ros2; then
    echo "ROS 2 installed"
else
    echo "ROS 2 not installed"
fi
