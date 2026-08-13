#!/bin/bash

# Define workspace directories
WORKSPACE_DIR="/home/charuka/Documents/uni pro"

echo "=========================================================="
echo "      TurtleBot3 Burger Camera Obstacle Detector Suite    "
echo "=========================================================="

# 1. Start Gazebo simulation in a new terminal window
echo "Step 1: Starting Gazebo simulation in a new window..."
gnome-terminal --title="Gazebo Simulation" -- bash -c "'$WORKSPACE_DIR/launch_simulation.sh'; exec bash"

# Wait a few seconds for Gazebo to initialize
echo "Waiting 5 seconds for Gazebo to load up..."
sleep 5

# 2. Start YOLO Obstacle Detector in a new terminal window
echo "Step 2: Starting YOLO Obstacle Detector in a new window..."
gnome-terminal --title="YOLO Obstacle Detector" -- bash -c "'$WORKSPACE_DIR/launch_detector.sh'; exec bash"

# Wait a moment
sleep 1

# 3. Start Teleop Keyboard in the current window (foreground)
echo "Step 3: Launching Keyboard Teleop in this terminal window..."
echo "----------------------------------------------------------"
echo "CLICK HERE and use the keyboard layout to drive the robot!"
echo "=========================================================="
"$WORKSPACE_DIR/launch_teleop.sh"
