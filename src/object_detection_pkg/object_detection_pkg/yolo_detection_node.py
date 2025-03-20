import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import numpy as np
import cv2  # Needed for image processing

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # Declare sim time parameter
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Read the sim time parameter
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f"use_sim_time set to: {self.use_sim_time}")

        # Set the parameter for ROS 2 clock sync
        # self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, self.use_sim_time)])

        # ROS2 Publisher
        self.detection_pub = self.create_publisher(Detection2DArray, '/yolo_detections', 10)

        # Image Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',  
            self.image_callback,
            10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load YOLO model (ensure path is correct)
        self.model = torch.hub.load('src/object_detection_pkg/object_detection_pkg/yolov5', 
                            'custom', path='src/object_detection_pkg/object_detection_pkg/best.pt', source='local')



    def image_callback(self, msg):
        """ Callback function for image frames """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def process_frame(self, image, header):
        """ Processes an image frame with YOLO and publishes bounding boxes """
        results = self.model(image)  # Run YOLO detection
        msg = Detection2DArray()
        msg.header = header  # Use original image timestamp
        msg.header.frame_id = "camera_link"

        # Process detections
        for det in results.xyxy[0]:  # YOLOv5 returns [x_min, y_min, x_max, y_max, conf, class]
            x_min, y_min, x_max, y_max, conf, cls = det.tolist()

            detection = Detection2D()
            detection.bbox.center.x = (x_min + x_max) / 2  # Compute center x
            detection.bbox.center.y = (y_min + y_max) / 2  # Compute center y
            detection.bbox.size_x = x_max - x_min  # Width
            detection.bbox.size_y = y_max - y_min  # Height

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cls)  # Object class ID
            hypothesis.score = float(conf)  # Confidence score
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        # Publish detections
        self.detection_pub.publish(msg)
        self.get_logger().info(f"Published {len(msg.detections)} detections.")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
