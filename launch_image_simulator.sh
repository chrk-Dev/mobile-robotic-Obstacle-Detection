#!/bin/bash
# launch_image_simulator.sh
# ──────────────────────────────────────────────────────────────────────────────
# Runs the image_obstacle_simulator node which feeds random COCO training images
# (from dataset/train/images/) into /camera/image_raw so the ObstacleDetector
# can detect objects WITHOUT needing Gazebo running.
#
# Usage:
#   ./launch_image_simulator.sh                        # default 1 Hz
#   ./launch_image_simulator.sh --hz 2.0               # 2 images/sec
#   ./launch_image_simulator.sh --hz 0.5 --no-loop     # 0.5 Hz, no repeat
# ──────────────────────────────────────────────────────────────────────────────

WORKSPACE_DIR="/home/charuka/Documents/uni pro"
TB3_WORKSPACE_DIR="/home/charuka/turtlebot3_ws"

# Source ROS 2 and both workspaces
source /opt/ros/jazzy/setup.bash
source "$TB3_WORKSPACE_DIR/install/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"

# ── Parse optional arguments ──────────────────────────────────────────────────
PUBLISH_HZ="1.0"
LOOP="true"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --hz)    PUBLISH_HZ="$2"; shift 2 ;;
        --no-loop) LOOP="false"; shift ;;
        *) echo "Unknown argument: $1"; shift ;;
    esac
done

echo "=================================================="
echo "  Image Obstacle Simulator (no Gazebo needed!)"
echo "  Images dir : $WORKSPACE_DIR/dataset/train/images"
echo "  Publish Hz : $PUBLISH_HZ"
echo "  Loop       : $LOOP"
echo "  Topic      : /camera/image_raw"
echo "=================================================="

ros2 run turtlebot_obstacle_detection image_obstacle_simulator \
    --ros-args \
    -p images_dir:="$WORKSPACE_DIR/dataset/train/images" \
    -p publish_hz:=$PUBLISH_HZ \
    -p loop:=$LOOP
