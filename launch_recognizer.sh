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
echo " Starting YOLO Object Recognizer & 3D Map Marker Node..."
echo " Subscribed to: /camera/image_raw, /scan, /tf"
echo " Publishing to: /recognized_objects_markers"
echo "=========================================================="

ros2 run turtlebot_obstacle_detection object_recognizer
