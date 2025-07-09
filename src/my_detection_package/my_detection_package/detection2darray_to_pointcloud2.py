import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import pyrealsense2 as rs

class Detection2DArrayToPointCloud2(Node):
    def __init__(self):
        super().__init__('detection2darray_to_pointcloud2')
        self.sub = self.create_subscription(Detection2DArray, '/yolo_bounding_boxes', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/yolo_detections', 10)
        
        # Initialize RealSense pipeline for depth information
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        
        # Get depth intrinsics
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def cb(self, msg):
        points = []
        
        # Get current depth frame
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            
            if not depth_frame:
                self.get_logger().warn("No depth frame available")
                return
                
            depth_image = np.asanyarray(depth_frame.get_data())
        except Exception as e:
            self.get_logger().error(f"Error getting depth frame: {e}")
            return

        for detection in msg.detections:
            # Get 2D pixel coordinates from detection
            cx = int(detection.bbox.center.position.x)
            cy = int(detection.bbox.center.position.y)
            
            # Ensure coordinates are within image bounds
            if 0 <= cx < 640 and 0 <= cy < 480:
                # Get depth value at detection center
                Z = depth_image[cy, cx] * self.depth_scale
                
                # Only process if we have valid depth
                if Z > 0 and Z < 10.0:  # Valid depth range 0-10m
                    # Convert pixel coordinates to 3D world coordinates
                    X, Y, _ = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [cx, cy], Z)
                    
                    # Add point to list (in camera coordinate frame)
                    points.append([X, Y, Z])

        if not points:
            return

        points_np = np.array(points, dtype=np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points_np),
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            is_bigendian=False,
            point_step=12,
            row_step=12 * len(points_np),
            data=points_np.tobytes(),
            is_dense=True
        )
        self.pub.publish(pc2_msg)
        self.get_logger().info(f"Published {len(points)} points to /yolo_detections")

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Detection2DArrayToPointCloud2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()