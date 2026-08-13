#!/bin/bash

# Define workspace directories
WORKSPACE_DIR="/home/charuka/Documents/uni pro"
TB3_WORKSPACE_DIR="/home/charuka/turtlebot3_ws"

echo "=========================================================="
echo "  TurtleBot3 Burger - Nav2 Navigation & Object Suite      "
echo "=========================================================="

# 1. Start Gazebo Simulation
echo "Step 1: Launching Gazebo 3D Simulation..."
gnome-terminal --title="1. Gazebo Simulation" -- bash -c "'$WORKSPACE_DIR/launch_simulation.sh'; exec bash"

# Wait for Gazebo to load models
echo "Waiting 7 seconds for Gazebo simulation to load..."
sleep 7

# 2. Start SLAM Toolbox & Nav2 Navigation Stack
echo "Step 2: Launching SLAM & Nav2 Stack..."
gnome-terminal --title="2. SLAM & Nav2 Navigation Stack" -- bash -c "'$WORKSPACE_DIR/launch_nav2_stack.sh'; exec bash"

# Wait for Nav2 servers to initialize
echo "Waiting 6 seconds for Nav2 to activate..."
sleep 6

# 3. Start YOLO Object Recognizer & 3D Map Marker Publisher
echo "Step 3: Launching YOLO Object Recognizer & 3D Map Marker Publisher..."
gnome-terminal --title="3. YOLO Object Recognizer" -- bash -c "'$WORKSPACE_DIR/launch_recognizer.sh'; exec bash"

sleep 2

# 4. Launch RViz2 Visualization
echo "Step 4: Launching RViz2 Visualizer..."
gnome-terminal --title="4. RViz2 Visualizer" -- bash -c "source /opt/ros/jazzy/setup.bash && rviz2 -d '$TB3_WORKSPACE_DIR/src/turtlebot3/turtlebot3_navigation2/rviz/tb3_navigation2.rviz'; exec bash"

sleep 3

# 5. Launch Autonomous Nav2 Object Explorer in foreground
echo "Step 5: Starting Autonomous Nav2 Object Explorer Sweep..."
echo "----------------------------------------------------------"
"$WORKSPACE_DIR/launch_explorer.sh"
