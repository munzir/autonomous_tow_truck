import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import torch
import pyrealsense2 as rs
import numpy as np
import time  # To measure latency

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.publisher_ = self.create_publisher(String, 'obstacle_info', 10)
        self.bridge = CvBridge()

        # Load YOLOv5 model (adjust model path if necessary)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n')  # Lightweight YOLOv5 Nano

        # Initialize RealSense pipeline and alignment for depth data
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Camera resolution
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Camera resolution
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # Timer to periodically capture frames
        self.timer = self.create_timer(0.1, self.capture_frame)

        self.profile = self.pipeline.get_active_profile()
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.width_t = 0.8
        self.height_t = 1.9

    def capture_frame(self):
        # Record the timestamp when the frame is captured
        start_capture_time = time.time()

        # Wait for frames and align the depth frame with the color frame
        frames = self.pipeline.wait_for_frames()  
        aligned_frames = self.align.process(frames)

        # Get the aligned depth and color frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            self.get_logger().error('Failed to capture depth or color frame')
            return

        # Convert color frame to OpenCV format
        color_image = np.asanyarray(color_frame.get_data())

        # Record timestamp when detection starts
        # detection_start_time = time.time()

        # Perform object detection and draw bounding boxes
        obstacle_detected, annotated_frame, distance = self.detect_obstacle(color_image, depth_frame)

        # Record timestamp when detection finishes
        detection_end_time = time.time()

        # Calculate detection latency (time from capture to detection)
        capture_and_detection_latency = detection_end_time - start_capture_time
        # detection_latency = detection_end_time - detection_start_time

        # Log latencies
        # self.get_logger().info(f"Capture to Detection Latency: {capture_and_detection_latency:.4f} seconds")
        # self.get_logger().info(f"Detection Latency: {detection_latency:.4f} seconds")

        # Publish obstacle status if detected
        # if obstacle_detected:    
            # msg = String()
            # msg.data = f"Obstacle detected at distance: {distance:.2f}m"
            # self.publisher_.publish(msg)


        if distance:    
            msg = String()
            brake_end_time = time.time()
            brake_latency = brake_end_time - start_capture_time
            msg.data = f"Apply brakes and latency is: {brake_latency:.4f} seconds"
            self.publisher_.publish(msg)
            # print(f"Capture to brake command Latency: {brake_latency:.4f} seconds")


        # Display the camera stream with bounding boxes
        cv2.imshow("Camera Stream", annotated_frame)
        cv2.waitKey(1)  # Wait for 1 ms to update the window

    def detect_obstacle(self, color_image, depth_frame):

        results = self.model(color_image)  # Perform YOLOv5 inference
        obstacle_detected = False
        range_within = 0  # Default to no obstacle within range

        for *box, conf, cls in results.pred[0]:  # Iterate over detections
            x1, y1, x2, y2 = map(int, box)  # Coordinates of the bounding box
            label = f"{results.names[int(cls)]} {conf:.2f}"  # Label with class name and confidence
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box
            obstacle_detected = True  
            
            # Get center point of the bounding box
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Get depth at the center point
            Z = depth_frame.get_distance(center_x, center_y)

            # Deproject to 3D camera coordinates
            X, Y, Z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], Z)
            # print("3D Camera Coordinates:", X, Y, Z)

            # Determine corner coordinates for horizontal checks
            if X < 0:  # Left side
                corner_x, corner_y, corner_z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x2, y2], Z)
            else:  # Right side
                corner_x, corner_y, corner_z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x1, y1], Z)

            # Check horizontal constraints
            if ((corner_x > -self.width_t / 2 - 0.1) or (corner_x < self.width_t / 2 + 0.1)) and (corner_y > -self.height_t/2 - 0.3) and (Z < 9):
                print("Brake")
                range_within = 1
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw bounding box
            else:
                print("Continue")
                range_within = 0
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box

            # Annotate distance on the image
            cv2.putText(color_image, f"Dist: {Z:.2f}m", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(color_image, f"X: {X:.2f}m, Y: {Y:.2f}m", (x1-20, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return obstacle_detected, color_image, range_within

    def destroy_node(self):
        # Stop the RealSense pipeline
        self.pipeline.stop()
        cv2.destroyAllWindows()  # Close all OpenCV windows
        super().destroy_node()  # Call the base class destroy_node method

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
