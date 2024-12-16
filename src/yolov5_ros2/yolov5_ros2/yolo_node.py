# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import torch
# import os

# class YoloNode(Node):
#     def __init__(self):
#         super().__init__('yolo_node')
        
#         # Subscriber: subscribing to the drone's camera feed
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/image_raw',  # The topic from the drone
#             self.listener_callback,
#             10)
        
#         # Publisher: publishing the processed image
#         self.publisher = self.create_publisher(Image, '/camera/yolov5_output', 10)
        
#         self.bridge = CvBridge()
        
#         # Load YOLOv5 model architecture from Ultralytics repo
#         self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=False)

#         # Load custom weights manually using torch.load
#         custom_weights_path = './best.pt'  # Make sure the path is correct
#         if os.path.exists(custom_weights_path):
#             self.model.load_state_dict(torch.load(custom_weights_path, map_location=torch.device('cpu'))['model'].state_dict())
#             self.get_logger().info(f'Custom YOLOv5 weights loaded from: {custom_weights_path}')
#         else:
#             self.get_logger().error(f'Custom YOLOv5 weights not found at: {custom_weights_path}')
        
#         self.get_logger().info('YOLOv5 node has been initialized')

#     def listener_callback(self, msg):
#         # Convert ROS 2 Image message to OpenCV image
#         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
#         # Run YOLOv5 detection
#         results = self.model(cv_image)
#         results.print()  # Print YOLOv5 results in the console

#         # Render the results on the image
#         rendered_image = results.render()[0]

#         # Display the result locally using OpenCV
#         cv2.imshow('YOLOv5 Detection', rendered_image)
#         cv2.waitKey(1)
        
#         # Convert the rendered image back to ROS 2 Image message
#         output_msg = self.bridge.cv2_to_imgmsg(rendered_image, encoding="bgr8")
        
#         # Publish the processed image
#         self.publisher.publish(output_msg)
#         self.get_logger().info('Published processed image with detections')

# def main(args=None):
#     rclpy.init(args=args)
#     node = YoloNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
