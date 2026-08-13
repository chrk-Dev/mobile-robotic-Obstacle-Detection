#!/usr/bin/env python3

import sys
import time
import math
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ANSI Colors
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[92m"
COLOR_RED = "\033[91m"
COLOR_YELLOW = "\033[93m"
COLOR_CYAN = "\033[96m"
COLOR_MAGENTA = "\033[95m"
COLOR_BOLD = "\033[1m"

def euler_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )

class AutonomousNav2Explorer(Node):
    def __init__(self):
        super().__init__('autonomous_nav2_explorer')

        self.recognized_objects_list = []
        self.objects_sub = self.create_subscription(
            String,
            '/recognized_objects',
            self.objects_callback,
            10
        )
        self.get_logger().info(f"{COLOR_CYAN}Autonomous Nav2 Object Explorer Node Initialized.{COLOR_RESET}")

    def objects_callback(self, msg):
        try:
            self.recognized_objects_list = json.loads(msg.data)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    explorer_node = AutonomousNav2Explorer()
    navigator = BasicNavigator()

    print(f"\n{COLOR_BOLD}=========================================================={COLOR_RESET}")
    print(f"{COLOR_BOLD}   AUTONOMOUS NAV2 OBJECT NAVIGATION & RECOGNITION        {COLOR_RESET}")
    print(f"{COLOR_BOLD}=========================================================={COLOR_RESET}\n")

    # Set initial pose if using AMCL / localization
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -2.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    print(f"{COLOR_CYAN}[NAV2] Waiting for Nav2 Stack to become active...{COLOR_RESET}")
    navigator.waitUntilNav2Active()
    print(f"{COLOR_GREEN}[NAV2] Nav2 Stack is fully ACTIVE and ready for navigation!{COLOR_RESET}\n")

    # Defined inspection route covering all obstacle regions in the 3D world
    # Format: (x, y, yaw_deg, label)
    inspection_waypoints = [
        (-2.5, -2.5,  45, "Sector 1: South-West Quad"),
        (-2.5,  0.0,   0, "Sector 2: West Mid"),
        (-2.5,  2.5, -45, "Sector 3: North-West Quad"),
        ( 0.0,  2.5, -90, "Sector 4: North Mid"),
        ( 2.5,  2.5,-135, "Sector 5: North-East Quad"),
        ( 2.5,  0.0, 180, "Sector 6: East Mid"),
        ( 2.5, -2.5, 135, "Sector 7: South-East Quad"),
        ( 0.0, -2.5,  90, "Sector 8: South Mid"),
        # Outer perimeter inspection sweep
        (-5.0, -3.5,  30, "Outer Perimeter SW"),
        (-5.0,  3.5, -30, "Outer Perimeter NW"),
        ( 5.0,  3.5,-150, "Outer Perimeter NE"),
        ( 5.0, -3.5, 150, "Outer Perimeter SE"),
        ( 0.0,  0.0,   0, "Return to Home Origin")
    ]

    total_wp = len(inspection_waypoints)

    for idx, (x, y, yaw_deg, label) in enumerate(inspection_waypoints, 1):
        yaw_rad = math.radians(yaw_deg)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = euler_to_quaternion(yaw_rad)

        print(f"{COLOR_CYAN}[NAV2 {idx}/{total_wp}]{COLOR_RESET} Navigating to {COLOR_BOLD}{label}{COLOR_RESET} at (X={x:.1f}, Y={y:.1f})...")
        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            rclpy.spin_once(explorer_node, timeout_sec=0.1)
            feedback = navigator.getFeedback()
            if feedback and i % 20 == 0:
                dist_remaining = feedback.distance_remaining
                print(f"   -> Progress: Distance remaining to waypoint: {dist_remaining:.2f} m")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"{COLOR_GREEN}[NAV2 SUCCESS]{COLOR_RESET} Arrived at {label}! Conducting camera scan...")
            # Spin node for 2.5 seconds to scan objects in camera FOV
            scan_end = time.time() + 2.5
            while time.time() < scan_end:
                rclpy.spin_once(explorer_node, timeout_sec=0.1)

            # Print current recognized objects summary
            objs = explorer_node.recognized_objects_list
            if objs:
                print(f"   {COLOR_MAGENTA}Current Recognized Objects Total: {len(objs)}{COLOR_RESET}")
                for obj in objs:
                    print(f"      • ID #{obj['id']}: {obj['class'].upper()} (Conf: {obj['conf']*100:.0f}%, Map: [{obj['x']}, {obj['y']}])")
            else:
                print(f"   Scanning area...")
        elif result == TaskResult.CANCELED:
            print(f"{COLOR_YELLOW}[NAV2 CANCELED]{COLOR_RESET} Goal canceled for {label}.")
        elif result == TaskResult.FAILED:
            print(f"{COLOR_RED}[NAV2 FAILED]{COLOR_RESET} Failed to reach {label}. Re-routing to next waypoint...")

        print("----------------------------------------------------------")

    # Print Final Summary Table
    print(f"\n{COLOR_BOLD}=========================================================={COLOR_RESET}")
    print(f"{COLOR_BOLD}   AUTONOMOUS INSPECTION SWEEP COMPLETE                   {COLOR_RESET}")
    print(f"{COLOR_BOLD}=========================================================={COLOR_RESET}")
    objs = explorer_node.recognized_objects_list
    print(f"{COLOR_GREEN}Total Objects Recognized & Mapped in 3D: {len(objs)}{COLOR_RESET}\n")

    for obj in objs:
        print(f"  • ID #{obj['id']:02d} | Class: {obj['class'].upper():14s} | Confidence: {obj['conf']*100:5.1f}% | Map Pos: (X={obj['x']:6.2f}, Y={obj['y']:6.2f})")

    print(f"\n{COLOR_CYAN}All recognized 3D markers are displayed in RViz2 under topic '/recognized_objects_markers'{COLOR_RESET}\n")

    explorer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
