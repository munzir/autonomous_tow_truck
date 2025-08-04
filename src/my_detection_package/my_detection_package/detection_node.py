import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import numpy as np
import torch
import pyrealsense2 as rs
import cv2

class YoloDetectionAndPointCloudNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.marker_pub = self.create_publisher(Marker, '/detection_points_marker', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/yolo_detections', 10)

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        # Get frames
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            self.get_logger().warn('No depth or color frame available')
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Run YOLO detection
        results = self.model(color_image)
        points = []
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link_optical'

        for *box, conf, cls in results.pred[0]:
            x1, y1, x2, y2 = map(int, box)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(x2, 639), min(y2, 479)
            if x2 <= x1 or y2 <= y1:
                continue
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            Z = depth_image[cy, cx] * self.depth_scale
            if Z >= 0 and Z < 3.0:
                X, Y, _ = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [cx, cy], Z)
                points.append([X, Y, Z])

            # Draw bounding box for visualization
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{int(cls)}: {conf:.2f}"
            cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Optional: publish marker
            marker = Marker()
            marker.header = header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(cx)
            marker.pose.position.y = float(cy)
            marker.pose.position.z = 0.0
            marker.scale.x = marker.scale.y = marker.scale.z = 10.0
            marker.color.r = 1.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)

        # Always publish PointCloud2 - empty if no obstacles detected
        if points:
            points_np = np.array(points, dtype=np.float32)
            self.get_logger().info(f"Published {len(points)} points to /yolo_detections")
        else:
            points_np = np.array([], dtype=np.float32).reshape(0, 3)  # Empty array
            self.get_logger().debug("Published empty pointcloud to /yolo_detections (no obstacles detected)")
        
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
        self.pc_pub.publish(pc2_msg)

        # Show the camera feed with bounding boxes
        cv2.imshow("Camera Feed", color_image)
        cv2.waitKey(1)

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionAndPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()