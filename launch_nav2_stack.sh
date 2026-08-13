#!/bin/bash

# Define workspace directories
WORKSPACE_DIR="/home/charuka/Documents/uni pro"
TB3_WORKSPACE_DIR="/home/charuka/turtlebot3_ws"

# Source ROS 2 system and workspaces
source /opt/ros/jazzy/setup.bash
source "$TB3_WORKSPACE_DIR/install/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

export TURTLEBOT3_MODEL=burger_cam

echo "=========================================================="
echo " Starting SLAM Toolbox & Nav2 Navigation Stack..."
echo "=========================================================="

# Launch SLAM Toolbox and Nav2 simultaneously
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true &
PID_NAV2=$!

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &
PID_SLAM=$!

# Wait for subprocesses
wait $PID_NAV2 $PID_SLAM
