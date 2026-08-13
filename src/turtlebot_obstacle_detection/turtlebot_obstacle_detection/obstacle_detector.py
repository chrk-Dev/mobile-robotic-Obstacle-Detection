#!/usr/bin/env python3

import os
import sys

# Ensure virtualenv libraries are loaded first so onnxruntime and numpy version match
venv_path = "/home/charuka/Documents/uni pro/venv/lib/python3.12/site-packages"
if venv_path not in sys.path:
    sys.path.insert(0, venv_path)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError
import onnxruntime as ort

# ANSI Color Codes for beautiful console reporting
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[92m"
COLOR_RED = "\033[91m"
COLOR_YELLOW = "\033[93m"
COLOR_CYAN = "\033[96m"

# Normalization constants (must match yololite training)
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

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Declare parameters for easy customization
        primary_model = '/home/charuka/Documents/uni pro/yololite/runs/export/2/model_decoded_nms.onnx'
        fallback_model = '/home/charuka/Documents/uni pro/yololite/yololite/runs/export/1/model_decoded_nms.onnx'
        default_model = primary_model if os.path.exists(primary_model) else fallback_model

        self.declare_parameter('model_path', default_model)
        self.declare_parameter('conf_threshold', 0.20)
        
        self.declare_parameter('roi_ymin', 0.20)  # Percentage from top of image for ROI start (expanded from 0.50)
        self.declare_parameter('roi_ymax', 0.98)  # Percentage from top of image for ROI end (expanded from 0.95)
        self.declare_parameter('roi_xmin', 0.10)  # Percentage from left of image for ROI start (expanded from 0.25)
        self.declare_parameter('roi_xmax', 0.90)  # Percentage from left of image for ROI end (expanded from 0.75)
        self.declare_parameter('roi_min_overlap_ratio', 0.15) # Minimum overlap ratio to trigger ROI obstacle

        # Fetch parameter values
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        
        self.roi_ymin_ratio = self.get_parameter('roi_ymin').get_parameter_value().double_value
        self.roi_ymax_ratio = self.get_parameter('roi_ymax').get_parameter_value().double_value
        self.roi_xmin_ratio = self.get_parameter('roi_xmin').get_parameter_value().double_value
        self.roi_xmax_ratio = self.get_parameter('roi_xmax').get_parameter_value().double_value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Publisher for obstacle status (useful for emergency braking/autonomy)
        self.alert_pub = self.create_publisher(Bool, '/obstacle_alert', 10)
        # Publisher for detailed object detection summary text
        self.detected_objects_pub = self.create_publisher(String, '/detected_objects', 10)

        # Subscriber to camera image topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Initialize ONNX Runtime session
        self.get_logger().info(f"{COLOR_CYAN}Loading ONNX model from {self.model_path}...{COLOR_RESET}")
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        so.intra_op_num_threads = 2
        
        try:
            self.sess = ort.InferenceSession(self.model_path, sess_options=so, providers=["CPUExecutionProvider"])
            self.in_name = self.sess.get_inputs()[0].name
            self.out_names = [o.name for o in self.sess.get_outputs()]
            self.get_logger().info(f"{COLOR_GREEN}ONNX model loaded successfully.{COLOR_RESET}")
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model at {self.model_path}, trying fallback...")
            try:
                self.model_path = fallback_model
                self.sess = ort.InferenceSession(self.model_path, sess_options=so, providers=["CPUExecutionProvider"])
                self.in_name = self.sess.get_inputs()[0].name
                self.out_names = [o.name for o in self.sess.get_outputs()]
                self.get_logger().info(f"{COLOR_GREEN}Fallback ONNX model loaded successfully.{COLOR_RESET}")
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
            self.get_logger().info(f"{COLOR_CYAN}Model uses 2-class obstacle mode: ['Obstacle', 'Static Obstacle']{COLOR_RESET}")
        else:
            self.class_names = [
                'person', 'bicycle', 'car', 'motorcycle',
                'dog', 'cat', 'chair', 'couch',
                'dining table', 'potted plant', 'backpack', 'suitcase',
                'tree'
            ]
            self.get_logger().info(f"{COLOR_CYAN}Model uses multi-class mode (13 categories){COLOR_RESET}")

        # Beautiful class colors for bounding boxes (BGR format)
        self.class_colors = [
            (255, 99, 71),   # Tomato
            (30, 144, 255),  # Dodger Blue
            (50, 205, 50),   # Lime Green
            (255, 215, 0),   # Gold
            (255, 69, 0),    # Orange Red
            (186, 85, 211),  # Medium Orchid
            (218, 112, 214), # Orchid
            (75, 0, 130),    # Indigo
            (244, 164, 96),  # Sandy Brown
            (46, 139, 87),   # Sea Green
            (112, 128, 144), # Slate Grey
            (222, 184, 135), # Burly Wood
            (34, 139, 34)    # Forest Green
        ]

        # Check if GUI display is available
        self.display_available = os.environ.get('DISPLAY') is not None
        if self.display_available:
            self.get_logger().info(f"{COLOR_CYAN}Display detected. Initializing OpenCV visual window (640x480)...{COLOR_RESET}")
            cv2.namedWindow("YOLO Obstacle Detector (with ROI)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("YOLO Obstacle Detector (with ROI)", 640, 480)
        else:
            self.get_logger().info(f"{COLOR_YELLOW}No display detected. Running in headless console-only mode.{COLOR_RESET}")

        self.get_logger().info(f"{COLOR_GREEN}YOLO Obstacle Detector Node Initialized. Subscribed to /camera/image_raw{COLOR_RESET}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Fetch parameter values dynamically on each frame to support dynamic tuning
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.roi_ymin_ratio = self.get_parameter('roi_ymin').get_parameter_value().double_value
        self.roi_ymax_ratio = self.get_parameter('roi_ymax').get_parameter_value().double_value
        self.roi_xmin_ratio = self.get_parameter('roi_xmin').get_parameter_value().double_value
        self.roi_xmax_ratio = self.get_parameter('roi_xmax').get_parameter_value().double_value
        min_overlap_ratio = self.get_parameter('roi_min_overlap_ratio').get_parameter_value().double_value

        h0, w0 = cv_image.shape[:2]

        # Calculate bounding box coordinates for ROI
        ymin_roi, ymax_roi = int(h0 * self.roi_ymin_ratio), int(h0 * self.roi_ymax_ratio)
        xmin_roi, xmax_roi = int(w0 * self.roi_xmin_ratio), int(w0 * self.roi_xmax_ratio)

        # 1. Preprocess the image for yololite (exported at 320x320)
        img_size = 320
        lb, scale, (padx, pady) = letterbox(cv_image, img_size)

        im = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        im = (im - MEAN) / STD
        im = np.transpose(im, (2, 0, 1))[None]  # [1, 3, 320, 320]

        # 2. Run ONNX Inference
        try:
            outputs = self.sess.run(self.out_names, {self.in_name: im})
            detections = outputs[0][0]  # Shape: [300, 6] -> [x1, y1, x2, y2, conf, class_id]
        except Exception as e:
            self.get_logger().error(f"Inference failed: {str(e)}")
            return

        # 3. Postprocess and filter predictions
        boxes_px = []
        scores_pad = []
        classes = []

        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            if conf > self.conf_threshold:
                # Map bounding box back to original coordinate space
                bx1 = (x1 - padx) / scale
                by1 = (y1 - pady) / scale
                bx2 = (x2 - padx) / scale
                by2 = (y2 - pady) / scale
                
                # Clip coordinates to image boundaries
                bx1 = max(0, min(w0 - 1, int(bx1)))
                by1 = max(0, min(h0 - 1, int(by1)))
                bx2 = max(0, min(w0 - 1, int(bx2)))
                by2 = max(0, min(h0 - 1, int(by2)))
                
                boxes_px.append([bx1, by1, bx2, by2])
                scores_pad.append(float(conf))
                classes.append(int(class_id))

        # 4. Filter obstacles and check if they overlap with the ROI path
        obstacle_detected = False
        roi_obstacles_details = []
        all_detections_details = []

        for b, s, c in zip(boxes_px, scores_pad, classes):
            bx1, by1, bx2, by2 = b
            class_name = self.class_names[c] if c < len(self.class_names) else 'object'
            
            box_area = max(1, (bx2 - bx1) * (by2 - by1))
            ix1 = max(bx1, xmin_roi)
            iy1 = max(by1, ymin_roi)
            ix2 = min(bx2, xmax_roi)
            iy2 = min(by2, ymax_roi)

            inter_area = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            overlap_ratio = inter_area / box_area

            # Bottom center check (ground contact point of object)
            bc_x = (bx1 + bx2) / 2.0
            bc_y = by2
            bc_in_roi = (xmin_roi <= bc_x <= xmax_roi) and (ymin_roi <= bc_y <= ymax_roi)

            # Object is in ROI path if bottom-center touches ROI OR intersection area ratio >= min_overlap_ratio
            overlap = bc_in_roi or (overlap_ratio >= min_overlap_ratio)
            item_desc = f"{class_name.capitalize()} ({s*100.0:.0f}%)"
            all_detections_details.append(item_desc)

            if overlap:
                obstacle_detected = True
                roi_obstacles_details.append(item_desc)

            # Draw detected object bounding box
            box_color = (0, 0, 255) if overlap else self.class_colors[c % len(self.class_colors)]
            thickness = 3 if overlap else 2
            cv2.rectangle(cv_image, (bx1, by1), (bx2, by2), box_color, thickness, cv2.LINE_AA)
            
            # Create a filled badge background behind label for high visibility
            label_text = f" {class_name.upper()} {s*100:.0f}%" + (" [IN ROI] " if overlap else " ")
            (tw, th), baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            badge_y1 = max(0, by1 - th - 8)
            badge_y2 = max(th + 8, by1)
            cv2.rectangle(cv_image, (bx1, badge_y1), (bx1 + tw, badge_y2), box_color, cv2.FILLED)
            cv2.putText(cv_image, label_text, (bx1, max(th + 3, by1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Publish the boolean alert
        alert_msg = Bool()
        alert_msg.data = obstacle_detected
        self.alert_pub.publish(alert_msg)

        # Publish detailed string summary of detected objects
        obj_msg = String()
        if obstacle_detected:
            obj_msg.data = f"ROI OBSTACLE: {', '.join(roi_obstacles_details)}"
        elif all_detections_details:
            obj_msg.data = f"IN VIEW: {', '.join(all_detections_details)}"
        else:
            obj_msg.data = "CLEAR: No objects detected"
        self.detected_objects_pub.publish(obj_msg)

        # Report to the console
        if obstacle_detected:
            details_str = ", ".join(roi_obstacles_details)
            print(f"{COLOR_RED}[WARNING] OBSTACLE IN PATH! {COLOR_RESET} Object(s): {COLOR_RED}{details_str}{COLOR_RESET}")
        elif all_detections_details:
            details_str = ", ".join(all_detections_details)
            print(f"{COLOR_YELLOW}[INFO] Objects in view (outside ROI): {COLOR_RESET} {details_str}")
        else:
            print(f"{COLOR_GREEN}[CLEAR] Path is clear. {COLOR_RESET} No objects detected.")

        # Visualization (if DISPLAY is available)
        if self.display_available:
            # Draw the ROI boundary rectangle on the original image
            roi_color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
            cv2.rectangle(cv_image, (xmin_roi, ymin_roi), (xmax_roi, ymax_roi), roi_color, 2, cv2.LINE_AA)
            cv2.putText(cv_image, "ROI PATH", (xmin_roi + 5, ymin_roi + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi_color, 2, cv2.LINE_AA)
            
            # Top HUD Banner across image header
            banner_h = 36
            overlay = cv_image.copy()
            if obstacle_detected:
                banner_color = (0, 0, 180) # Red HUD
                banner_text = f"DANGER: OBSTACLE IN ROI -> {', '.join(roi_obstacles_details)}"
            elif all_detections_details:
                banner_color = (0, 140, 200) # Amber/Yellow HUD
                banner_text = f"NOTICE: IN VIEW -> {', '.join(all_detections_details)} (Path Clear)"
            else:
                banner_color = (0, 140, 0) # Green HUD
                banner_text = "SAFE: PATH CLEAR - No obstacles detected"

            cv2.rectangle(overlay, (0, 0), (w0, banner_h), banner_color, cv2.FILLED)
            cv2.addWeighted(overlay, 0.85, cv_image, 0.15, 0, cv_image)
            cv2.putText(cv_image, banner_text, (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            # High-quality bicubic display scaling if image size is below target display resolution
            target_w, target_h = 1024, 768
            if w0 < target_w or h0 < target_h:
                display_img = cv2.resize(cv_image, (target_w, target_h), interpolation=cv2.INTER_CUBIC)
            else:
                display_img = cv_image

            # Show the visual feed
            cv2.imshow("YOLO Obstacle Detector (with ROI)", display_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nNode stopped by User. Exiting...")
    finally:
        if node.display_available:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
