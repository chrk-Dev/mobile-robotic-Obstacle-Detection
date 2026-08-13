# turtlebot_obstacle_detection

A ROS 2 (Jazzy) package for real-time obstacle detection, object recognition, and autonomous navigation on a **TurtleBot3 Burger** (camera-equipped) using a custom-trained **YoloLite ONNX** model.

---

## Overview

This package bridges a lightweight on-device YOLO model with the ROS 2 navigation stack (Nav2). It provides five nodes that work together:

| Node | Executable | Purpose |
|------|-----------|---------|
| `ObstacleDetector` | `obstacle_detector` | Real-time ONNX inference on `/camera/image_raw`; publishes obstacle alerts |
| `ObjectRecognizerNav` | `object_recognizer` | Fuses camera + LiDAR to localize detected objects in the map frame |
| `AutonomousNav2Explorer` | `autonomous_nav2_explorer` | Nav2-based waypoint sweep that triggers camera scans at each sector |
| `ImageObstacleSimulator` | `image_obstacle_simulator` | Replaces Gazebo camera with shuffled COCO dataset images for offline testing |
| `TeleopWASD` | `teleop_wasd` | Keyboard teleoperation (W/A/S/D) for manual robot driving |

---

## Architecture

```
/camera/image_raw  ──►  ObstacleDetector  ──►  /obstacle_alert  (std_msgs/Bool)
                                           ──►  /detected_objects (std_msgs/String)

/camera/image_raw  ──►  ObjectRecognizerNav ──►  /recognized_objects        (std_msgs/String)
/scan              ──►  (LiDAR depth fusion)  ──►  /recognized_objects_markers (visualization_msgs/MarkerArray)

/recognized_objects ──►  AutonomousNav2Explorer ──►  Nav2 goal poses (waypoint sweep)

dataset/train/images/ ──►  ImageObstacleSimulator ──►  /camera/image_raw
```

---

## Prerequisites

- **ROS 2 Jazzy** installed and sourced
- **TurtleBot3 workspace** built at `~/turtlebot3_ws`
- **Nav2** (`nav2_simple_commander`)
- Python virtualenv at `<workspace>/venv` with:
  - `onnxruntime`
  - `opencv-python`
  - `numpy`
  - `cv_bridge`
- A trained **YoloLite ONNX model** (see [YoloLite README](../../yololite/README.md))

---

## Build

```bash
cd "/home/charuka/Documents/uni pro"
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
colcon build --packages-select turtlebot_obstacle_detection
source install/setup.bash
```

---

## Nodes

### 1. `obstacle_detector`

Subscribes to `/camera/image_raw`, runs YoloLite ONNX inference at 320×320, and checks detections against a configurable ROI (Region of Interest) representing the robot's forward path.

**Published Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/obstacle_alert` | `std_msgs/Bool` | `true` when any object overlaps the ROI |
| `/detected_objects` | `std_msgs/String` | Human-readable summary of all detections |

**Subscribed Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Live camera feed |

**Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | Auto-detected from `runs/export/` | Path to decoded ONNX model |
| `conf_threshold` | `0.20` | Minimum detection confidence |
| `roi_ymin` | `0.20` | ROI top boundary (fraction of image height) |
| `roi_ymax` | `0.98` | ROI bottom boundary |
| `roi_xmin` | `0.10` | ROI left boundary (fraction of image width) |
| `roi_xmax` | `0.90` | ROI right boundary |
| `roi_min_overlap_ratio` | `0.15` | Minimum box-ROI overlap to trigger alert |

**Launch**

```bash
./launch_detector.sh
# or manually:
ros2 run turtlebot_obstacle_detection obstacle_detector \
  --ros-args -p conf_threshold:=0.25 -p roi_ymin:=0.3
```

---

### 2. `object_recognizer`

Fuses camera detections with LiDAR scan data to estimate each detected object's 2D position in the `map` frame using TF2 transforms. Maintains a persistent object database.

**Published Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/recognized_objects` | `std_msgs/String` | JSON array of recognized objects with map coordinates |
| `/recognized_objects_markers` | `visualization_msgs/MarkerArray` | RViz2 markers for each object |

**Subscribed Topics**

| Topic | Type |
|-------|------|
| `/camera/image_raw` | `sensor_msgs/Image` |
| `/scan` | `sensor_msgs/LaserScan` |

**Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | Auto-detected | Path to decoded ONNX model |
| `conf_threshold` | `0.25` | Detection confidence threshold |
| `camera_fov_deg` | `62.2` | Horizontal camera FOV (Raspberry Pi Burger Cam) |

**Launch**

```bash
./launch_recognizer.sh
```

---

### 3. `autonomous_nav2_explorer`

Autonomously navigates through 13 pre-defined waypoints covering the TurtleBot3 world, pausing at each sector to conduct a camera scan and log recognized objects.

**Waypoint Sectors**

The inspection route covers 8 inner sectors and 4 outer-perimeter positions, returning home at completion.

**Subscribed Topics**

| Topic | Type |
|-------|------|
| `/recognized_objects` | `std_msgs/String` |

**Launch**

```bash
./launch_explorer.sh
# Requires Nav2 stack to be active first:
./launch_nav2_stack.sh
```

---

### 4. `image_obstacle_simulator`

Publishes dataset images on `/camera/image_raw` at a configurable rate, acting as a drop-in replacement for the Gazebo camera. Useful for testing the detector without a live simulation.

**Published Topics**

| Topic | Type |
|-------|------|
| `/camera/image_raw` | `sensor_msgs/Image` |

**Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `images_dir` | `<workspace>/dataset/train/images` | Folder of `.jpg`/`.png` images |
| `publish_hz` | `1.0` | Images published per second |
| `loop` | `true` | Shuffle and repeat endlessly |
| `show_window` | `true` | Show OpenCV preview (requires `DISPLAY`) |

**Launch**

```bash
./launch_image_simulator.sh
# or manually:
ros2 run turtlebot_obstacle_detection image_obstacle_simulator \
  --ros-args -p publish_hz:=2.0 -p images_dir:=/path/to/images
```

---

### 5. `teleop_wasd`

Keyboard teleoperation node. Click the terminal and use `W/A/S/D` to drive, `Q/E` to adjust speed.

```bash
./launch_teleop.sh
```

---

## Quick Start — Full Pipeline

### Option A: Gazebo Simulation + Detector + Teleop

```bash
# Launch everything in separate terminals automatically
./launch_all.sh
```

This opens:
1. **Gazebo** with the randomized obstacle world (`turtlebot3_world`)
2. **YOLO Obstacle Detector** node
3. **Keyboard Teleop** in the foreground terminal

### Option B: Image Simulator (no Gazebo)

Use this when running without a display or Gazebo:

```bash
# Terminal 1 – publish dataset images
./launch_image_simulator.sh

# Terminal 2 – run the detector
./launch_detector.sh
```

### Option C: Full Nav2 Autonomous Sweep

```bash
# Terminal 1
./launch_simulation.sh          # Gazebo world

# Terminal 2
./launch_nav2_all.sh            # Nav2 stack + AMCL

# Terminal 3
./launch_recognizer.sh          # Object Recognizer + LiDAR fusion

# Terminal 4
./launch_explorer.sh            # Autonomous waypoint sweep
```

---

## Model

The package uses a **YoloLite `edge_n`** model exported to ONNX (`decoded_nms` format) at **320×320** input resolution. The model supports two operating modes determined automatically at startup:

| Mode | Classes | Use case |
|------|---------|----------|
| 2-class | `Obstacle`, `Static Obstacle` | Custom obstacle-only training |
| 13-class | `person, bicycle, car, motorcycle, dog, cat, chair, couch, dining table, potted plant, backpack, suitcase, tree` | COCO subset training |

Model path is resolved automatically:
1. `yololite/runs/export/2/model_decoded_nms.onnx` (primary)
2. `yololite/yololite/runs/export/1/model_decoded_nms.onnx` (fallback)

Override with the `model_path` parameter at launch.

See the [YoloLite README](../../yololite/README.md) for training and export instructions.

---

## Package Structure

```
src/turtlebot_obstacle_detection/
├── turtlebot_obstacle_detection/
│   ├── obstacle_detector.py          # YOLO + ROI obstacle alerting
│   ├── object_recognizer_nav.py      # Camera + LiDAR 3D object mapping
│   ├── autonomous_nav2_explorer.py   # Nav2 autonomous waypoint sweep
│   ├── image_obstacle_simulator.py   # Dataset image publisher (Gazebo replacement)
│   └── teleop_wasd.py                # WASD keyboard teleoperation
├── package.xml
├── setup.py
└── README.md
```

---

## Troubleshooting

**`No module named 'onnxruntime'`** — Ensure the virtualenv path is set correctly in each node (top of file) and the venv is populated.

**`Failed to load ONNX model`** — Check that a trained model exists at the expected path. Run the YoloLite export script first.

**Black/empty camera window** — Confirm Gazebo is running and the `/camera/image_raw` topic is being published: `ros2 topic hz /camera/image_raw`

**Nav2 not activating** — Launch Nav2 before the explorer node and wait for the "Nav2 Stack is fully ACTIVE" message.
