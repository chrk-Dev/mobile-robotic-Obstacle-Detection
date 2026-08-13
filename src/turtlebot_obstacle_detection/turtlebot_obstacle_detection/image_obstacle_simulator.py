#!/usr/bin/env python3
"""
image_obstacle_simulator.py
============================
Replaces the Gazebo simulation obstacle stream with random real images from
the COCO training dataset (dataset/train/images/).

Each image is published on /camera/image_raw at a configurable rate so that
the existing ObstacleDetector node can process them exactly as if they came
from a live camera.

Usage (after colcon build):
    ros2 run turtlebot_obstacle_detection image_obstacle_simulator

Parameters (set via --ros-args -p <name>:=<value>):
    images_dir  – Path to folder containing .jpg / .png images
                  Default: /home/charuka/Documents/uni pro/dataset/train/images
    publish_hz  – How many images per second to publish  (Default: 1.0)
    loop        – If True, shuffle and repeat the list endlessly (Default: True)
    show_window – Show each image in an OpenCV window before publishing
                  (Default: True, suppressed when DISPLAY is absent)
"""

import os
import sys
import glob
import random

# Ensure virtualenv libraries are loaded so cv2 / numpy versions match
venv_path = "/home/charuka/Documents/uni pro/venv/lib/python3.12/site-packages"
if venv_path not in sys.path:
    sys.path.insert(0, venv_path)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ── ANSI colours ─────────────────────────────────────────────────────────────
C_RESET  = "\033[0m"
C_GREEN  = "\033[92m"
C_RED    = "\033[91m"
C_YELLOW = "\033[93m"
C_CYAN   = "\033[96m"
C_BOLD   = "\033[1m"

DEFAULT_IMAGES_DIR = "/home/charuka/Documents/uni pro/dataset/train/images"


class ImageObstacleSimulator(Node):
    """
    Publishes random dataset images on /camera/image_raw, acting as a drop-in
    replacement for the Gazebo camera.  The ObstacleDetector node reads from
    the same topic, so no other code changes are needed.
    """

    def __init__(self):
        super().__init__('image_obstacle_simulator')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('images_dir', DEFAULT_IMAGES_DIR)
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('show_window', True)

        self.images_dir  = self.get_parameter('images_dir').get_parameter_value().string_value
        publish_hz       = self.get_parameter('publish_hz').get_parameter_value().double_value
        self.loop        = self.get_parameter('loop').get_parameter_value().bool_value
        show_window_pref = self.get_parameter('show_window').get_parameter_value().bool_value

        # Display is only available when a screen is attached
        self.display_available = (os.environ.get('DISPLAY') is not None) and show_window_pref

        # ── Load image list ───────────────────────────────────────────────────
        patterns = [
            os.path.join(self.images_dir, '*.jpg'),
            os.path.join(self.images_dir, '*.jpeg'),
            os.path.join(self.images_dir, '*.png'),
        ]
        self.image_paths = []
        for p in patterns:
            self.image_paths.extend(glob.glob(p))

        if not self.image_paths:
            self.get_logger().error(
                f"{C_RED}No images found in: {self.images_dir}{C_RESET}"
            )
            raise RuntimeError(f"No images found in: {self.images_dir}")

        # Shuffle once at start
        random.shuffle(self.image_paths)
        self.total_images = len(self.image_paths)
        self.index        = 0

        self.get_logger().info(
            f"{C_GREEN}{C_BOLD}ImageObstacleSimulator{C_RESET} — "
            f"loaded {C_CYAN}{self.total_images}{C_RESET} images from "
            f"'{self.images_dir}'"
        )

        # ── ROS infrastructure ────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.pub    = self.create_publisher(Image, '/camera/image_raw', 10)

        period_sec = 1.0 / max(publish_hz, 0.01)   # guard against zero
        self.timer = self.create_timer(period_sec, self._publish_next_image)

        self.get_logger().info(
            f"Publishing at {C_CYAN}{publish_hz:.2f} Hz{C_RESET} "
            f"on {C_CYAN}/camera/image_raw{C_RESET}"
        )
        if self.display_available:
            self.get_logger().info(
                f"{C_CYAN}OpenCV preview window enabled.{C_RESET}"
            )
        else:
            self.get_logger().info(
                f"{C_YELLOW}No display – running headless.{C_RESET}"
            )

    # ── Timer callback ────────────────────────────────────────────────────────
    def _publish_next_image(self):
        if self.index >= len(self.image_paths):
            if self.loop:
                # Re-shuffle and restart
                random.shuffle(self.image_paths)
                self.index = 0
                self.get_logger().info(
                    f"{C_YELLOW}All images shown — reshuffling and looping.{C_RESET}"
                )
            else:
                self.get_logger().info(
                    f"{C_GREEN}All {self.total_images} images published. "
                    f"Shutting down.{C_RESET}"
                )
                self.timer.cancel()
                rclpy.shutdown()
                return

        img_path = self.image_paths[self.index]
        self.index += 1

        # Load image
        bgr = cv2.imread(img_path)
        if bgr is None:
            self.get_logger().warning(
                f"{C_YELLOW}Could not read image: {img_path} — skipping.{C_RESET}"
            )
            return

        filename = os.path.basename(img_path)
        progress = f"[{self.index}/{len(self.image_paths)}]"

        print(
            f"{C_CYAN}{progress}{C_RESET} Publishing obstacle image: "
            f"{C_BOLD}{filename}{C_RESET}"
        )

        # Optional preview window
        if self.display_available:
            preview = bgr.copy()
            # Overlay file name on the preview
            cv2.putText(
                preview, filename, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2
            )
            cv2.putText(
                preview, progress, (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1
            )
            cv2.imshow("Image Obstacle Simulator", preview)
            cv2.waitKey(1)

        # Convert and publish
        try:
            ros_img            = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = 'camera_link'
            self.pub.publish(ros_img)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to convert/publish image '{filename}': {exc}"
            )


# ── Entry-point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ImageObstacleSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nNode stopped by user. Exiting...")
    finally:
        if node.display_available:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
