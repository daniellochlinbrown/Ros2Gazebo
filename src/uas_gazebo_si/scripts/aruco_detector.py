#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Add a delay to give Gazebo time to start publishing
        self.get_logger().info('Waiting for Gazebo to start publishing...')
        time.sleep(5)  # Adjust the delay time as needed

        # Define the QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribe to the camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Camera topic from Gazebo
            self.image_callback,
            qos_profile
        )

        # Publishers: processed image, marker pose, and visualization marker
        self.image_pub = self.create_publisher(Image, '/aruco_detector/image_processed', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_detector/pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/aruco_detector/marker', 10)

        # Initialize CVBridge to convert between ROS and OpenCV
        self.bridge = CvBridge()

        # Aruco dictionary and detector parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
        self.parameters = aruco.DetectorParameters_create()

        # Camera calibration parameters (as calculated earlier)
        self.camera_matrix = np.array([[554.256, 0, 320], [0, 554.256, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

        # Marker size (adjust based on your actual marker size in meters)
        self.marker_size = 0.05  # 5 cm marker

        self.get_logger().info('Aruco Detector node has started')

    def image_callback(self, data):
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

                # Estimate pose of each marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    # Draw axis on the marker
                    aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                    # Publish the pose of the marker
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'camera_frame'
                    
                    pose_msg.pose.position.x = tvecs[i][0][0]
                    pose_msg.pose.position.y = tvecs[i][0][1]
                    pose_msg.pose.position.z = tvecs[i][0][2]

                    # Convert rotation vector (rvec) to quaternion for orientation
                    rmat, _ = cv2.Rodrigues(rvecs[i])
                    quaternion = self.rotation_matrix_to_quaternion(rmat)

                    pose_msg.pose.orientation.x = quaternion[0]
                    pose_msg.pose.orientation.y = quaternion[1]
                    pose_msg.pose.orientation.z = quaternion[2]
                    pose_msg.pose.orientation.w = quaternion[3]

                    # Publish the pose to RViz
                    self.pose_pub.publish(pose_msg)

                    # Create and publish a marker to RViz at the detected location
                    marker_msg = Marker()
                    marker_msg.header.stamp = self.get_clock().now().to_msg()
                    marker_msg.header.frame_id = 'camera_frame'

                    marker_msg.ns = 'aruco_marker'
                    marker_msg.id = int(ids[i][0])
                    marker_msg.type = Marker.CUBE  # You can change this to ARROW, SPHERE, etc.
                    marker_msg.action = Marker.ADD

                    marker_msg.pose = pose_msg.pose  # Same pose as the detected marker

                    marker_msg.scale.x = 0.05  # Adjust marker size
                    marker_msg.scale.y = 0.05
                    marker_msg.scale.z = 0.05

                    marker_msg.color.a = 1.0  # Set opacity
                    marker_msg.color.r = 0.0
                    marker_msg.color.g = 1.0  # Green marker
                    marker_msg.color.b = 0.0

                    # Publish the marker to RViz
                    self.marker_pub.publish(marker_msg)

            # Convert the OpenCV image back to a ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(processed_image_msg)

            # Display the result for debugging
            cv2.imshow('Aruco Marker Detection', cv_image)
            cv2.waitKey(3)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def rotation_matrix_to_quaternion(self, rmat):
        # Convert rotation matrix to quaternion
        qx, qy, qz, qw = np.zeros((4,))
        trace = np.trace(rmat)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rmat[2][1] - rmat[1][2]) * s
            qy = (rmat[0][2] - rmat[2][0]) * s
            qz = (rmat[1][0] - rmat[0][1]) * s
        else:
            if rmat[0][0] > rmat[1][1] and rmat[0][0] > rmat[2][2]:
                s = 2.0 * np.sqrt(1.0 + rmat[0][0] - rmat[1][1] - rmat[2][2])
                qw = (rmat[2][1] - rmat[1][2]) / s
                qx = 0.25 * s
                qy = (rmat[0][1] + rmat[1][0]) / s
                qz = (rmat[0][2] + rmat[2][0]) / s
            elif rmat[1][1] > rmat[2][2]:
                s = 2.0 * np.sqrt(1.0 + rmat[1][1] - rmat[0][0] - rmat[2][2])
                qw = (rmat[0][2] - rmat[2][0]) / s
                qx = (rmat[0][1] + rmat[1][0]) / s
                qy = 0.25 * s
                qz = (rmat[1][2] + rmat[2][1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rmat[2][2] - rmat[0][0] - rmat[1][1])
                qw = (rmat[1][0] - rmat[0][1]) / s
                qx = (rmat[0][2] + rmat[2][0]) / s
                qy = (rmat[1][2] + rmat[2][1]) / s
                qz = 0.25 * s
        return [qx, qy, qz, qw]

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
