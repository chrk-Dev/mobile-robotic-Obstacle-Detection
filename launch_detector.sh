#!/bin/bash

# Define workspace directories
WORKSPACE_DIR="/home/charuka/Documents/uni pro"
TB3_WORKSPACE_DIR="/home/charuka/turtlebot3_ws"

# Source ROS 2 system and both workspaces
source /opt/ros/jazzy/setup.bash
source "$TB3_WORKSPACE_DIR/install/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

# Set the TurtleBot3 model
export TURTLEBOT3_MODEL=burger_cam

echo "==============================================="
echo "Launching YOLO Obstacle Detector..."
echo "Subscribed to: /camera/image_raw"
echo "==============================================="

# Run the obstacle detector node
ros2 run turtlebot_obstacle_detection obstacle_detector
