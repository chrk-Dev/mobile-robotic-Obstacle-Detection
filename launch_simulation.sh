#!/bin/bash

# Define workspace directories
WORKSPACE_DIR="/home/charuka/Documents/uni pro"
TB3_WORKSPACE_DIR="/home/charuka/turtlebot3_ws"

# Source ROS 2 system and both workspaces
source /opt/ros/jazzy/setup.bash
source "$TB3_WORKSPACE_DIR/install/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

# Set the TurtleBot3 model to the camera-equipped Burger
export TURTLEBOT3_MODEL=burger_cam

# Randomize obstacles in the world map
python3 "$WORKSPACE_DIR/randomize_obstacles.py"

# Launch the world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

