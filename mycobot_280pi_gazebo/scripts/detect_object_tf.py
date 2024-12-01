#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2

class RedColorDetection(Node):
    def __init__(self):
        super().__init__('red_color_detection_node')
        
        self.get_logger().info("Starting color detection node")

        self.contour_image = '/contour_image'
        self.image_raw= '/camera_head/color/image_raw'
        self.point_cloud_topic = '/camera_head/depth/color/points'
        

        # Create a CvBridge object for converting ROS image to OpenCV image
        self.bridge = CvBridge()

        # Create TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Create a publisher for the contour image
        self.contour_image_pub = self.create_publisher(
            Image,
            self.contour_image,
            10
        )

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            self.image_raw,
            self.image_callback,
            10
        )
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
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
        
        # Red color detection in the image
        red_mask = self.detect_red_color(self.latest_image)
        
        # Find contours to calculate centroid
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        contour_image = self.latest_image.copy()
        
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter out small contours
                # Draw the contour on the image
                cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 6)
                
                # Calculate centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    print("Centroid:", cx, cy)
                    #Draw centroid
                    cv2.circle(contour_image, (cx,cy), 10, (0,255,0), -1)

                    # Get depth information at the centroid position
                    try:
                        depth = self.get_depth_at_centroid(cx, cy, pc_data)
                        self.publish_tf(depth)
                    except:
                        print("Exception in getting depth")
                        pass

                    # Publish the contour image
                    self.publish_contour_image(contour_image)

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
       
        depth_point = pc_points[cy, cx]
        return depth_point


    def publish_tf(self, depth):
        """Publish the transform for detected object."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_head_depth_frame'
        transform.child_frame_id = 'detect_object'

        # Assuming the depth from point cloud is the Z coordinate
        transform.transform.translation.x = float(depth[0])
        transform.transform.translation.y = float(depth[1])
        transform.transform.translation.z = float(depth[2])

        # Identity rotation (no rotation assumed for simplicity)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)





    def publish_contour_image(self, image):
        """Publish the image with contours as a ROS topic."""
        try:
            # Convert OpenCV image back to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.contour_image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Failed to publish contour image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RedColorDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
