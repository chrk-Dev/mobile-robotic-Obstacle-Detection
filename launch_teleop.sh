#!/bin/bash

# Define workspace directories
WORKSPACE_DIR="/home/charuka/Documents/uni pro"
TB3_WORKSPACE_DIR="/home/charuka/turtlebot3_ws"

# Source ROS 2 system and both workspaces
source /opt/ros/jazzy/setup.bash
source "$TB3_WORKSPACE_DIR/install/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

echo "==============================================="
echo "Launching Keyboard Teleop (model agnostic)..."
echo "Publishing TwistStamped to: /cmd_vel"
echo "Use WASD / Arrow Keys / etc. to drive the robot."
echo "==============================================="

# Run the teleop node, ensuring it publishes stamped Twist messages
ros2 run turtlebot_obstacle_detection teleop_wasd --ros-args -p stamped:=true




