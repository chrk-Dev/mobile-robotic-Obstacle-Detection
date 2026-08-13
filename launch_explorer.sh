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
echo " Launching Autonomous Nav2 Object Navigation Explorer..."
echo " Driving around all objects and recognizing them in 3D..."
echo "=========================================================="

ros2 run turtlebot_obstacle_detection autonomous_nav2_explorer
