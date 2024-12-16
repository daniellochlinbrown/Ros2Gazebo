#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ManualImagePublisher(Node):
    def __init__(self):
        super().__init__('manual_image_publisher')

        # Create a publisher to the camera topic
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # Initialize the CV Bridge
        self.bridge = CvBridge()

        # Load an image from file (adjust the path to your image)
        image_path = '/home/daniel/ros2_ws/src/aruco_detector/resource/aruco_10.png'
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            self.get_logger().error(f"Failed to load image at {image_path}")
            return

        # Convert the OpenCV image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # Publish the image
        self.image_pub.publish(image_msg)
        self.get_logger().info('Published image')

def main(args=None):
    rclpy.init(args=args)
    manual_image_publisher = ManualImagePublisher()

    try:
        rclpy.spin_once(manual_image_publisher)
    except KeyboardInterrupt:
        pass

    manual_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
