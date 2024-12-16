#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Or use a more appropriate message type
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Create a subscriber to the camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic name
            self.image_callback,
            10
        )

        # Create a publisher to publish detection results
        self.detection_pub = self.create_publisher(String, '/aruco_detection', 10)  # Change message type as needed

        # Initialize the CV Bridge
        self.bridge = CvBridge()

        # Load Aruco dictionary and detector parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()

        self.get_logger().info('Aruco Detector node has started')

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect the markers in the grayscale image
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            # If markers are detected, draw them on the image and publish the result
            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.get_logger().info(f'Detected marker IDs: {ids.flatten()}')

                # Publish the detected marker IDs (you can publish more data if needed)
                marker_ids = ', '.join(map(str, ids.flatten()))
                detection_msg = String()
                detection_msg.data = f"Detected Aruco markers: {marker_ids}"
                self.detection_pub.publish(detection_msg)

            # Display the result
            cv2.imshow('Aruco Marker Detection', cv_image)
            cv2.waitKey(3)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()

    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass

    aruco_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
