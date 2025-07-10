#!/bin/bash

echo "===================="
echo "🛡️  Starting check_script.sh"
echo "===================="
echo

# Source the ROS 2 setup script
echo "[INFO] Sourcing ROS 2 setup..."
source /opt/ros/humble/setup.bash
if [ -f /home/drims/static/drims2_ws/install/setup.bash ]; then
    echo "[INFO] Sourcing static workspace setup..."
    source /home/drims/static/drims2_ws/install/setup.bash
fi

echo
echo "--------------------"
echo "🔍 Checking environment"
echo "--------------------"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if static workspace exists
if [ -d /home/drims/static/drims2_ws/src ]; then
    echo "[OK] static drims ws exists"
else
    echo "[WARN] static drims ws does not exist"
fi

# Check if ROS 2 is installed
if command_exists ros2; then
    echo "[OK] ROS 2 installed"
else
    echo "[ERROR] ROS 2 not installed"
fi

# Check if user workspace is mounted
if [ -d /home/drims/drims_ws/src ]; then
    echo "[OK] user drims ws mounted"
else
    echo "[WARN] user drims ws does not exist"
fi

echo
echo "===================="
echo "✅ Finished check_script.sh"
echo "===================="

