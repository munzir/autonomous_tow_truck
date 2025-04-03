import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
import cv2
from cv_bridge import CvBridge
import torch
# import pyrealsense2 as rs
import numpy as np
import struct
import time  # To measure latency

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None  # RealSense SDK is optional, only needed if using hardware

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        self.declare_parameter('use_hardware', True)
        self.use_hardware = self.get_parameter('use_hardware').value
        self.bridge = CvBridge()
        
        self.declare_parameter('use_sim_time', False)  # Add this
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.bridge = CvBridge()
        
        self.string_publisher = self.create_publisher(String, 'obstacle_info', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'camera_obstacles', 10)
        self.cx, self.cy, self.fx, self.fy = None, None, None, None  # Initialize intrinsics
        
        if self.use_hardware and rs is not None:
            self.setup_realsense_pipeline()
        elif not self.use_hardware and self.use_sim_time:
            self.get_logger().info("Running with simulated time")
            self.get_logger().info("Using simulated camera")
            self.camera_info_sub = self.create_subscription(
                CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
            self.image_sub = self.create_subscription(
                Image, '/camera/rgb/image_raw', self.image_callback, 10)
            self.depth_sub = self.create_subscription(
                Image, '/camera/depth/image_raw', self.depth_callback, 10)
            self.latest_depth_frame = None
        
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

    def setup_realsense_pipeline(self):
        # Initialize RealSense camera
        self.get_logger().info("Using RealSense Hardware")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.profile = self.pipeline.get_active_profile()
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        self.width_t = 0.8
        self.height_t = 1.9

        # Timer to periodically capture frames
        self.timer = self.create_timer(0.1, self.capture_frame)

    
    def capture_frame(self):
        start_capture_time = time.time()

        if self.use_hardware:
            # RealSense frame capture
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                self.get_logger().error('Failed to capture depth or color frame')
                return

            color_image = np.asanyarray(color_frame.get_data())

        else:
            # Gazebo Simulation: Ensure both images have been received
            if self.latest_depth_frame is None or self.latest_color_frame is None:
                return  # Wait until both images are available

            color_image = self.latest_color_frame
            depth_frame = self.latest_depth_frame  # This will be a NumPy array from ROS Image

        # Perform object detection
        obstacle_detected, annotated_frame, points = self.detect_obstacle(color_image, depth_frame)

        # Publish PointCloud2 data
        if points:
            self.publish_obstacles(points)

        # Publish brake message
        if obstacle_detected:
            msg = String()
            brake_end_time = time.time()
            brake_latency = brake_end_time - start_capture_time
            msg.data = f"Apply brakes and latency is: {brake_latency:.4f} seconds"
            self.string_publisher.publish(msg)

        # Display camera feed with bounding boxes
        cv2.imshow("Camera Stream", annotated_frame)
        cv2.waitKey(1)

    def detect_obstacle(self, color_image, depth_frame):
        results = self.model(color_image)
        obstacle_detected = False
        points = []  # To store 3D points for PointCloud2

        for *box, conf, cls in results.pred[0]:
            x1, y1, x2, y2 = map(int, box)
            label = f"{results.names[int(cls)]} {conf:.2f}"
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            obstacle_detected = True  

            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Get depth value
            if self.use_hardware:
                Z = depth_frame.get_distance(center_x, center_y)
                X, Y, Z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], Z)
            else:
                # Gazebo depth frame is a raw NumPy array (normalized)
                # depth_value = depth_frame[center_y, center_x]  # Depth at object center
                # Z = depth_value * 10  # Scale factor for depth (adjust as needed)
                Z = float(depth_frame[center_y, center_x])  # Gazebo depth values are in meters

                X = (center_x - self.cx) * Z / self.fx
                Y = (center_y - self.cy) * Z / self.fy
                self.get_logger().info(f"Gazebo Depth Image: Min={np.min(depth_frame)}, Max={np.max(depth_frame)}")


            if Z < 9:  # Only publish obstacles within 9m range
                points.append((X, Y, Z))

            cv2.putText(color_image, f"Dist: {Z:.2f}m", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(color_image, f"X: {X:.2f}m, Y: {Y:.2f}m", (x1-20, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return obstacle_detected, color_image, points

    def publish_obstacles(self, points):
        if not points:
            self.get_logger().warn("No valid points detected, skipping PointCloud2 publishing.")
            return

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = "camera_link"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud.height = 1  # Unordered point cloud
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12  # Each point is 3x float32 (4 bytes each)
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False  # Set False in case of invalid depth points

        # Convert list of points to binary format
        cloud.data = b''.join(struct.pack("fff", *p) for p in points)

        self.get_logger().info(f"Publishing {len(points)} points in PointCloud2")
        for i, p in enumerate(points[:5]):  # Print first 5 points
            self.get_logger().info(f"Point {i}: X={p[0]:.2f}, Y={p[1]:.2f}, Z={p[2]:.2f}")

        self.pointcloud_publisher.publish(cloud)
        self.get_logger().info(f"Published PointCloud2 with {len(points)} points")


    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

    def image_callback(self, msg):
        self.latest_color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.latest_depth_frame = np.array(depth_image, dtype=np.float32)  # Convert ROS image to NumPy

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]  # Focal length in x
        self.fy = msg.k[4]  # Focal length in y
        self.cx = msg.k[2]  # Principal point x
        self.cy = msg.k[5]  # Principal point y
        self.get_logger().info(f"Camera intrinsics set: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
