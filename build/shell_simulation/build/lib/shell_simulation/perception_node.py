#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Parameters
        self.declare_parameter('obstacle_distance_threshold', 3.0)  # meters
        self.declare_parameter('roi_x_start_ratio', 0.4) # Ratio of image width
        self.declare_parameter('roi_x_end_ratio', 0.6)   # Ratio of image width
        self.declare_parameter('roi_y_start_ratio', 0.4) # Ratio of image height
        self.declare_parameter('roi_y_end_ratio', 0.6)   # Ratio of image height
        
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').get_parameter_value().double_value
        self.roi_x_start_ratio = self.get_parameter('roi_x_start_ratio').get_parameter_value().double_value
        self.roi_x_end_ratio = self.get_parameter('roi_x_end_ratio').get_parameter_value().double_value
        self.roi_y_start_ratio = self.get_parameter('roi_y_start_ratio').get_parameter_value().double_value
        self.roi_y_end_ratio = self.get_parameter('roi_y_end_ratio').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Subscription to Depth Camera
        self.depth_subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/depth_middle/image',
            self.depth_callback,
            10)
        
        # Publisher for Obstacle Alert
        self.obstacle_alert_publisher = self.create_publisher(Bool, '/obstacle_alert', 10)
        
        self.get_logger().info(f"Perception Node started. Obstacle threshold: {self.obstacle_threshold}m")

    def depth_callback(self, msg):
        try:
            # CARLA depth images are typically 32FC1 (float32, 1 channel)
            # The values are in meters.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        height, width = cv_image.shape

        # Define ROI (Region of Interest) in the center
        roi_x_start = int(width * self.roi_x_start_ratio)
        roi_x_end = int(width * self.roi_x_end_ratio)
        roi_y_start = int(height * self.roi_y_start_ratio)
        roi_y_end = int(height * self.roi_y_end_ratio)
        
        # Ensure ROI coordinates are valid
        roi_x_start = max(0, roi_x_start)
        roi_x_end = min(width, roi_x_end)
        roi_y_start = max(0, roi_y_start)
        roi_y_end = min(height, roi_y_end)

        if roi_x_start >= roi_x_end or roi_y_start >= roi_y_end:
            self.get_logger().warn("Invalid ROI dimensions, check ratios.")
            obstacle_detected_msg = Bool()
            obstacle_detected_msg.data = False # Default to no obstacle if ROI is invalid
            self.obstacle_alert_publisher.publish(obstacle_detected_msg)
            return

        roi = cv_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        if roi.size == 0:
            self.get_logger().warn("ROI is empty.")
            obstacle_detected_msg = Bool()
            obstacle_detected_msg.data = False # Default to no obstacle if ROI is empty
            self.obstacle_alert_publisher.publish(obstacle_detected_msg)
            return
            
        # Calculate average depth in ROI, ignoring inf values
        valid_depths = roi[np.isfinite(roi) & (roi > 0)] # Consider only valid positive depth values
        
        obstacle_detected = False
        if valid_depths.size > 0:
            average_depth = np.mean(valid_depths)
            if average_depth < self.obstacle_threshold:
                obstacle_detected = True
                # self.get_logger().info(f"Obstacle detected at {average_depth:.2f}m")
        else:
            # self.get_logger().info("No valid depth points in ROI or all points are too far.")
            pass


        obstacle_detected_msg = Bool()
        obstacle_detected_msg.data = obstacle_detected
        self.obstacle_alert_publisher.publish(obstacle_detected_msg)

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()