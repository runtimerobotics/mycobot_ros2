#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
#from pcl import PointCloud
#import pcl
from sensor_msgs_py import point_cloud2

class RedColorDetection(Node):
    def __init__(self):
        super().__init__('red_color_detection_node')
        

        self.get_logger().info("Starting color detection node")

        # Create a CvBridge object for converting ROS image to OpenCV image
        self.bridge = CvBridge()
        
        # Create TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera_head/color/image_raw',
            self.image_callback,
            10
        )
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera_head/depth/color/points',
            self.pc_callback,
            10
        )
        
        self.latest_image = None
        self.latest_point_cloud = None
        
    def image_callback(self, msg):
        try:
            # Convert the image from ROS message to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def pc_callback(self, msg):
        if self.latest_image is None:
            self.get_logger().warn("No image received yet, skipping point cloud processing.")
            return
        
        # Convert point cloud message to numpy array
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True, reshape_organized_cloud=True)
        #pc_points = np.array(list(pc_data))
        #point = pc_data[200,200]
        #print("Point got it",point)
        # Red color detection in the image
        red_mask = self.detect_red_color(self.latest_image)
        
        # Find contours to calculate centroid
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter out small contours
                # Calculate centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    print("Centroid:",cx,cy)
                    # Get depth information at the centroid position
                    depth = self.get_depth_at_centroid(cx, cy, pc_data)
                    if depth is not None:
                        self.publish_tf(cx, cy, depth)

    def detect_red_color(self, image):
        """Detect red color in the image and return a binary mask."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        red_mask = mask1 | mask2
        return red_mask

    def get_depth_at_centroid(self, cx, cy, pc_points):
        """Get the depth (z-coordinate) at the centroid (cx, cy)."""
        # Define a small area around the centroid to gather depth data
        window_size = 0.001
        depth_values = []
        
        for point in pc_points:
            x, y, z = point

            #print(x,y,z)
            # Check if the point is within the region of interest around the centroid
            if abs(x - cx) < window_size and abs(y - cy) < window_size:
                print("Found a point")
                depth_values.append(z)
        
        if depth_values:
            # Return the median depth value to avoid outliers
            print("Depth values",depth_values)
            return np.median(depth_values)
        else:
            self.get_logger().warn(f"No depth found at centroid: ({cx}, {cy})")
            return None

    def publish_tf(self, cx, cy, depth):
        """Publish the transform for detected object."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_head_depth_optical_frame'
        transform.child_frame_id = 'detect_object'

        # Assuming the depth from point cloud is the Z coordinate
        transform.transform.translation.x = cx
        transform.transform.translation.y = cy
        transform.transform.translation.z = depth

        # Identity rotation (no rotation assumed for simplicity)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = RedColorDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
