#!/usr/bin/env python3

import os
import sys
import math
import json

# Ensure virtualenv libraries are loaded first so onnxruntime and numpy version match
venv_path = "/home/charuka/Documents/uni pro/venv/lib/python3.12/site-packages"
if venv_path not in sys.path:
    sys.path.insert(0, venv_path)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import onnxruntime as ort

import tf2_ros
from tf2_ros import TransformException

# ANSI Color Codes
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[92m"
COLOR_RED = "\033[91m"
COLOR_YELLOW = "\033[93m"
COLOR_CYAN = "\033[96m"
COLOR_MAGENTA = "\033[95m"

# Image Normalization Constants
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD  = np.array([0.229, 0.224, 0.225], np.float32)

def letterbox(im, new_size=320, color=(114, 114, 114)):
    h, w = im.shape[:2]
    scale = min(new_size / h, new_size / w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    im_resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top = (new_size - nh) // 2
    bottom = new_size - nh - top
    left = (new_size - nw) // 2
    right = new_size - nw - left
    im_padded = cv2.copyMakeBorder(im_resized, top, bottom, left, right,
                                   cv2.BORDER_CONSTANT, value=color)
    return im_padded, scale, (left, top)

class ObjectRecognizerNav(Node):
    def __init__(self):
        super().__init__('object_recognizer_nav')

        # Parameters
        primary_model = '/home/charuka/Documents/uni pro/yololite/runs/export/2/model_decoded_nms.onnx'
        fallback_model = '/home/charuka/Documents/uni pro/yololite/yololite/runs/export/1/model_decoded_nms.onnx'
        default_model = primary_model if os.path.exists(primary_model) else fallback_model

        self.declare_parameter('model_path', default_model)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('camera_fov_deg', 62.2) # Horizontal FOV for Raspberry Pi / Burger Cam
        
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.fov_rad = math.radians(self.get_parameter('camera_fov_deg').get_parameter_value().double_value)

        # CV Bridge
        self.bridge = CvBridge()

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage for latest scan
        self.latest_scan = None

        # Recognized objects database: dict key = object_id, value = {class_name, x, y, conf, last_seen}
        self.recognized_objects = {}
        self.next_object_id = 1

        # Publishers
        self.alert_pub = self.create_publisher(Bool, '/obstacle_alert', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/recognized_objects_markers', 10)
        self.objects_pub = self.create_publisher(String, '/recognized_objects', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Load ONNX model
        self.get_logger().info(f"{COLOR_CYAN}Loading ONNX model from {self.model_path}...{COLOR_RESET}")
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        so.intra_op_num_threads = 2

        try:
            self.sess = ort.InferenceSession(self.model_path, sess_options=so, providers=["CPUExecutionProvider"])
            self.in_name = self.sess.get_inputs()[0].name
            self.out_names = [o.name for o in self.sess.get_outputs()]
            self.get_logger().info(f"{COLOR_GREEN}ONNX Object Detection model loaded successfully.{COLOR_RESET}")
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model at {self.model_path}, trying fallback...")
            try:
                self.model_path = fallback_model
                self.sess = ort.InferenceSession(self.model_path, sess_options=so, providers=["CPUExecutionProvider"])
                self.in_name = self.sess.get_inputs()[0].name
                self.out_names = [o.name for o in self.sess.get_outputs()]
                self.get_logger().info(f"{COLOR_GREEN}Fallback ONNX Object Detection model loaded successfully.{COLOR_RESET}")
            except Exception as e_fallback:
                self.get_logger().error(f"Failed to load fallback ONNX model: {str(e_fallback)}")
                raise e_fallback

        # Determine class names automatically based on model output classes
        try:
            dummy_in = np.zeros((1, 3, 320, 320), dtype=np.float32)
            dummy_out = self.sess.run(self.out_names, {self.in_name: dummy_in})[0][0]
            max_cls_id = int(np.max(dummy_out[:, 5])) if len(dummy_out) > 0 else 0
        except Exception:
            max_cls_id = 1

        if max_cls_id <= 1:
            self.class_names = ['Obstacle', 'Static Obstacle']
            self.get_logger().info(f"{COLOR_CYAN}Model uses 2-class mode: ['Obstacle', 'Static Obstacle']{COLOR_RESET}")
        else:
            self.class_names = [
                'person', 'bicycle', 'car', 'motorcycle',
                'dog', 'cat', 'chair', 'couch',
                'dining table', 'potted plant', 'backpack', 'suitcase',
                'tree'
            ]
            self.get_logger().info(f"{COLOR_CYAN}Model uses multi-class mode (13 categories){COLOR_RESET}")

        # Class RGB colors for RViz and OpenCV (0.0 to 1.0 for RViz)
        self.class_colors_rgb = [
            (1.0, 0.2, 0.2),  # person - Red
            (0.2, 0.6, 1.0),  # bicycle - Blue
            (0.2, 0.9, 0.2),  # car - Green
            (1.0, 0.8, 0.0),  # motorcycle - Yellow
            (1.0, 0.4, 0.0),  # dog - Orange
            (0.8, 0.2, 0.8),  # cat - Purple
            (0.9, 0.4, 0.8),  # chair - Pink
            (0.4, 0.1, 0.6),  # couch - Dark Violet
            (0.8, 0.5, 0.2),  # dining table - Brown
            (0.1, 0.7, 0.4),  # potted plant - Sea Green
            (0.5, 0.5, 0.5),  # backpack - Grey
            (0.8, 0.7, 0.5),  # suitcase - Tan
            (0.15, 0.68, 0.25) # tree - Forest Green
        ]

        self.display_available = os.environ.get('DISPLAY') is not None
        self.get_logger().info(f"{COLOR_GREEN}Object Recognizer & 3D Nav Marker Node Started.{COLOR_RESET}")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_distance_for_angle(self, angle_rad):
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            return 1.5  # Fallback default distance in meters

        scan = self.latest_scan
        # Normalize angle to [-pi, pi]
        angle_rad = (angle_rad + math.pi) % (2 * math.pi) - math.pi

        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        index = int((angle_rad - angle_min) / angle_increment)
        num_ranges = len(scan.ranges)

        if 0 <= index < num_ranges:
            # Check a small window around index for valid distance
            window = 5
            valid_ranges = []
            for i in range(max(0, index - window), min(num_ranges, index + window + 1)):
                r = scan.ranges[i]
                if scan.range_min < r < scan.range_max and not math.isinf(r) and not math.isnan(r):
                    valid_ranges.append(r)
            if valid_ranges:
                return float(np.median(valid_ranges))

        return 1.5

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")
            return

        h0, w0 = cv_image.shape[:2]

        # 1. Preprocess image
        img_size = 320
        lb, scale, (padx, pady) = letterbox(cv_image, img_size)

        im = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        im = (im - MEAN) / STD
        im = np.transpose(im, (2, 0, 1))[None]

        # 2. Run ONNX Inference
        try:
            outputs = self.sess.run(self.out_names, {self.in_name: im})
            detections = outputs[0][0]
        except Exception as e:
            self.get_logger().error(f"Inference failed: {str(e)}")
            return

        # 3. Lookup TF robot pose in map frame
        map_pose = None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
            map_pose = (tx, ty, yaw)
        except TransformException:
            # Fallback to odom frame if map frame not yet available
            try:
                transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
                map_pose = (tx, ty, yaw)
            except TransformException:
                pass

        # 4. Process Detections
        detected_in_frame = []
        obstacle_alert = False

        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            if conf > self.conf_threshold:
                bx1 = max(0, min(w0 - 1, int((x1 - padx) / scale)))
                by1 = max(0, min(h0 - 1, int((y1 - pady) / scale)))
                bx2 = max(0, min(w0 - 1, int((x2 - padx) / scale)))
                by2 = max(0, min(h0 - 1, int((y2 - pady) / scale)))

                c_idx = int(class_id)
                class_name = self.class_names[c_idx] if c_idx < len(self.class_names) else 'object'

                # Calculate object center in image
                cx = (bx1 + bx2) / 2.0
                
                # Check if object is close in central region of image
                if 0.25 * w0 <= cx <= 0.75 * w0 and by2 > 0.5 * h0:
                    obstacle_alert = True

                # Compute bearing relative to robot forward (+X)
                normalized_x = (cx - w0 / 2.0) / (w0 / 2.0) # [-1, 1]
                bearing_rad = -normalized_x * (self.fov_rad / 2.0)

                # Get distance to object using LiDAR
                dist_m = self.get_distance_for_angle(bearing_rad)

                # Estimate 3D map coordinates if robot pose is known
                obj_map_x, obj_map_y = None, None
                if map_pose is not None:
                    rx, ry, ryaw = map_pose
                    obj_angle = ryaw + bearing_rad
                    obj_map_x = rx + dist_m * math.cos(obj_angle)
                    obj_map_y = ry + dist_m * math.sin(obj_angle)
                    self.update_recognized_object(class_name, float(conf), obj_map_x, obj_map_y, c_idx)

                detected_in_frame.append({
                    'class': class_name,
                    'confidence': float(conf),
                    'box': [bx1, by1, bx2, by2],
                    'distance': round(dist_m, 2),
                    'map_pos': [round(obj_map_x, 2), round(obj_map_y, 2)] if obj_map_x is not None else None
                })

                # Draw OpenCV bounding box
                r, g, b = self.class_colors_rgb[c_idx % len(self.class_colors_rgb)]
                bgr_color = (int(b * 255), int(g * 255), int(r * 255))
                cv2.rectangle(cv_image, (bx1, by1), (bx2, by2), bgr_color, 2)
                text = f"{class_name} {conf*100:.0f}% ({dist_m:.1f}m)"
                cv2.putText(cv_image, text, (bx1, max(15, by1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)

        # Publish Alert Status
        alert_msg = Bool()
        alert_msg.data = obstacle_alert
        self.alert_pub.publish(alert_msg)

        # Publish Recognized Objects JSON
        json_msg = String()
        json_msg.data = json.dumps(list(self.recognized_objects.values()))
        self.objects_pub.publish(json_msg)

        # Publish RViz 3D Markers
        self.publish_rviz_markers()

        # Log detections to console periodically
        if detected_in_frame:
            det_summary = ", ".join([f"{d['class']} ({d['confidence']*100:.0f}%, {d['distance']}m)" for d in detected_in_frame])
            print(f"{COLOR_MAGENTA}[RECOGNIZER]{COLOR_RESET} Frame Detections: {det_summary}")

        # Display window if available
        if self.display_available:
            status_str = f"Recognized Total: {len(self.recognized_objects)} Objects"
            cv2.putText(cv_image, status_str, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("Nav2 Object Recognizer (YOLO Lite)", cv_image)
            cv2.waitKey(1)

    def update_recognized_object(self, class_name, conf, x, y, class_id):
        # Merge detections within 1.2m radius of an existing recorded object
        merge_threshold = 1.2
        best_id = None
        min_dist = float('inf')

        for obj_id, obj_data in self.recognized_objects.items():
            if obj_data['class'] == class_name:
                dist = math.hypot(x - obj_data['x'], y - obj_data['y'])
                if dist < merge_threshold and dist < min_dist:
                    min_dist = dist
                    best_id = obj_id

        now_sec = self.get_clock().now().to_msg().sec

        if best_id is not None:
            # Update existing entry with moving average position
            prev = self.recognized_objects[best_id]
            prev['x'] = 0.7 * prev['x'] + 0.3 * x
            prev['y'] = 0.7 * prev['y'] + 0.3 * y
            prev['conf'] = max(prev['conf'], conf)
            prev['last_seen'] = now_sec
            prev['count'] += 1
        else:
            # Create new entry
            obj_id = self.next_object_id
            self.next_object_id += 1
            self.recognized_objects[obj_id] = {
                'id': obj_id,
                'class': class_name,
                'class_id': class_id,
                'conf': conf,
                'x': round(x, 2),
                'y': round(y, 2),
                'last_seen': now_sec,
                'count': 1
            }
            print(f"{COLOR_GREEN}[NEW OBJECT RECOGNIZED!]{COLOR_RESET} ID #{obj_id}: {class_name.upper()} at Map Position (X={x:.2f}, Y={y:.2f})")

    def publish_rviz_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for obj_id, obj in self.recognized_objects.items():
            r, g, b = self.class_colors_rgb[obj['class_id'] % len(self.class_colors_rgb)]

            # 1. 3D Shape Marker (Cylinder)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'recognized_objects_shapes'
            marker.id = obj_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(obj['x'])
            marker.pose.position.y = float(obj['y'])
            marker.pose.position.z = 0.4
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 0.8
            marker.color.r = float(r)
            marker.color.g = float(g)
            marker.color.b = float(b)
            marker.color.a = 0.75
            marker.lifetime.sec = 0 # Persistent
            marker_array.markers.append(marker)

            # 2. 3D Text Label Marker
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = stamp
            text_marker.ns = 'recognized_objects_text'
            text_marker.id = obj_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(obj['x'])
            text_marker.pose.position.y = float(obj['y'])
            text_marker.pose.position.z = 1.1
            text_marker.scale.z = 0.35 # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{obj['class'].upper()}\n({obj['conf']*100:.0f}%)"
            text_marker.lifetime.sec = 0
            marker_array.markers.append(text_marker)

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizerNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping Object Recognizer Node...")
    finally:
        if node.display_available:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
