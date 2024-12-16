#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Log that the node is waiting for the camera topic to be available
        self.get_logger().info('Waiting for the camera topic to be available...')
        
        # Wait for the topic to become available
        self.wait_for_topic('/camera/image_raw')

        # Create a subscriber to the camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic name
            self.image_callback,
            10
        )

        # Initialize the CV Bridge
        self.bridge = CvBridge()

        # Load Aruco dictionary and detector parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()

        self.get_logger().info('Aruco Detector node has started')

    def wait_for_topic(self, topic_name):
        """ Waits until the specified topic is available. """
        while not self.get_topic_names_and_types():
            self.get_logger().info(f'Waiting for {topic_name} topic...')
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=1))

    def image_callback(self, data):
        self.get_logger().info('Received an image')
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect the markers in the grayscale image
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            # If markers are detected, draw them on the image
            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.get_logger().info(f'Detected marker IDs: {ids.flatten()}')

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
        aruco_detector.get_logger().info('Aruco Detector node interrupted by user.')
    except Exception as e:
        aruco_detector.get_logger().error(f'An error occurred: {e}')
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        aruco_detector.get_logger().info('Aruco Detector node has been shut down.')

if __name__ == '__main__':
    main()
