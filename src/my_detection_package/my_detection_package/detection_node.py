import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import torch
import re
import pyrealsense2 as rs
import numpy as np
import time  # To measure latency'
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header  # Added Header import

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.publisher_ = self.create_publisher(String, 'obstacle_info', 10)
        # self.detection_publisher_ = self.create_publisher(String, '/yolo_detections', 10)
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, '/yolo_obstacles', 10)  # New publisher
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

        self.fx = 385
        self.fy = 385
        self.cx = 325
        self.cy = 239

    def bbox_to_pointcloud(self, bbox, depth_frame, header):
        """Convert YOLO bboxes to PointCloud2 with class information in intensity field"""
        point_cloud = []
        depth_image = np.asanyarray(depth_frame.get_data())

        match = re.search(r"Box: \((\d+), (\d+), (\d+), (\d+)\), Dist:\s*([\d.]+)m", bbox)
        if match:
            # Convert box coordinates to integers
            x_min, y_min, x_max, y_max = map(int, match.groups()[:4])
            # Convert distance to float
            z_dist = float(match.group(5))
        else:
            raise ValueError(f"Failed to parse bbox string: {bbox}")
            
        for y in range(y_min, y_max, 5):
            for x in range(x_min, x_max, 5):
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x]
                    if depth > 0:
                        z = float(depth)
                        x_3d = (x - self.cx) * z / self.fx
                        y_3d = (y - self.cy) * z / self.fy
                        # Store point with class as intensity (4th field)
                        point_cloud.append([x_3d, y_3d, z])
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        header.frame_id = "camera_depth_optical_frame"
        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(point_cloud),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 4 fields * 4 bytes
            row_step=12 * len(point_cloud),
            data=np.asarray(point_cloud, dtype=np.float32).tobytes()
        )
        
        return pc2_msg

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

        # Perform object detection and draw bounding boxes
        obstacle_detected, annotated_frame, distance = self.detect_obstacle(color_image, depth_frame)

        # Record timestamp when detection finishes
        detection_end_time = time.time()

        # Calculate detection latency (time from capture to detection)
        capture_and_detection_latency = detection_end_time - start_capture_time


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
        results = self.model(color_image)
        obstacle_detected = False
        range_within = 0

        # Create header with ROS 2 timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # ROS 2 way to get time
        header.frame_id = "camera_depth_optical_frame"  # Match your TF tree

        for *box, conf, cls in results.pred[0]:
            x1, y1, x2, y2 = map(int, box)
            label = f"{results.names[int(cls)]} {conf:.2f}"
            obstacle_detected = True

            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            Z = depth_frame.get_distance(center_x, center_y)
            X, Y, Z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], Z)

            # Horizontal logic (as before)
            if X < 0:
                corner_x, corner_y, corner_z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x2, y2], Z)
            else:
                corner_x, corner_y, corner_z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x1, y1], Z)

            if ((corner_x > -self.width_t / 2 - 0.1) or (corner_x < self.width_t / 2 + 0.1)) and (corner_y > -self.height_t/2 - 0.3) and (Z < 9):
                range_within = 1
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            else:
                range_within = 0
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(color_image, f"Dist: {Z:.2f}m", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(color_image, f"X: {X:.2f}m, Y: {Y:.2f}m", (x1 - 20, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # ==== NEW: Publish detection info ====
            detection_info = f"Box: ({x1}, {y1}, {x2}, {y2}), Dist: {Z:.2f}m"
            # print(depth_frame.shape)
            pc_msg =  self.bbox_to_pointcloud(detection_info, depth_frame, header)
            # self.detection_publisher_.publish(String(data=detection_info))
            if pc_msg:
                self.pointcloud_publisher_.publish(pc_msg)

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
   
