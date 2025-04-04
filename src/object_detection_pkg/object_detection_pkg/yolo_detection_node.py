import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import torch
import numpy as np
import message_filters  # For synchronized subscriptions

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.publisher_ = self.create_publisher(String, 'obstacle_info', 10)
        self.bridge = CvBridge()

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n')  # YOLOv5 Nano

        # Subscribe to RGB and Depth image topics using time synchronization
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)

        self.width_t = 0.8
        self.height_t = 1.9

    def image_callback(self, rgb_msg, depth_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        start_time = self.get_clock().now().nanoseconds / 1e9

        obstacle_detected, annotated_image, should_brake = self.detect_obstacle(color_image, depth_image)

        if should_brake:
            end_time = self.get_clock().now().nanoseconds / 1e9
            latency = end_time - start_time
            msg = String()
            msg.data = f"Apply brakes and latency is: {latency:.4f} seconds"
            self.publisher_.publish(msg)

        # Show annotated image
        cv2.imshow("Camera Stream", annotated_image)
        cv2.waitKey(1)

    def detect_obstacle(self, color_image, depth_image):
        results = self.model(color_image)
        obstacle_detected = False
        should_brake = False

        for *box, conf, cls in results.pred[0]:
            x1, y1, x2, y2 = map(int, box)
            label = f"{results.names[int(cls)]} {conf:.2f}"
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Extract depth in meters (depth_image is assumed to be in meters or mm depending on sim)
            Z = depth_image[center_y, center_x]

            if Z == 0 or np.isnan(Z):
                continue

            # Simple range check for obstacle (Z < 9m and within width and height constraints)
            if Z < 9:
                obstacle_detected = True
                should_brake = True
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(color_image, f"Dist: {Z:.2f}m", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                cv2.putText(color_image, f"Dist: {Z:.2f}m", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return obstacle_detected, color_image, should_brake

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
