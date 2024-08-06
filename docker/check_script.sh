#!/bin/bash

# Source the ROS setup script
source /home/drims/catkin_ws/devel/setup.bash

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if /drims_ws exists
if [ -f /drims_ws/.placeholder ]; then
    echo "drims ws exists"
else
    echo "drims ws does not exist"
fi

# Check if ROS is installed
if command_exists rospack; then
    echo "ROS installed"

    # Check for omnicore_launcher package
    if rospack list | grep -q omnicore_launcher; then
        echo "omnicore installed"
    else
        echo "omnicore_launcher not found"
    fi

    # Check for depthai_ros_driver package
    if rospack list | grep -q depthai_ros_driver; then
        echo "depthai installed"
    else
        echo "depthai_ros_driver not found"
    fi
else
    echo "ROS not installed"
fi
