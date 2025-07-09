import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, ObjectHypothesis, BoundingBox2D, Pose2D
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import torch
import pyrealsense2 as rs
import cv2
from threading import Lock
from collections import defaultdict

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        self.detection_publisher_ = self.create_publisher(
            Detection2DArray,
            '/yolo_bounding_boxes',
            10
        )
        self.marker_pub = self.create_publisher(Marker, '/detection_points_marker', 10)

        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

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

        self.obstacles = defaultdict(lambda: {'last_seen': self.get_clock().now(), 'visible': False, 'position': (0, 0, 0), 'class': None, 'bbox': (0, 0, 0, 0)})
        self.obstacle_timeout = 0.3
        self.obstacle_id_counter = 0

        self.frame_lock = Lock()
        self.timer = self.create_timer(0.1, self.capture_frame)

    def capture_frame(self):
        with self.frame_lock:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        results = self.model(color_image)
        detections = []
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link_optical"

        for *box, conf, cls in results.pred[0]:
            x1, y1, x2, y2 = map(int, box)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(x2, 639), min(y2, 479)
            if x2 <= x1 or y2 <= y1:
                continue
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            Z = depth_image[cy, cx] * self.depth_scale
            X, Y, _ = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [cx, cy], Z)

            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = float(cx)
            detection.bbox.center.position.y = float(cy)
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(cls))
            hyp.hypothesis.score = float(conf)
            detection.results.append(hyp)

            detections.append(detection)

            # Draw bounding box on the color image
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{int(cls)}: {conf:.2f}"
            cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        msg_array = Detection2DArray()
        msg_array.header = header
        msg_array.detections = detections
        self.detection_publisher_.publish(msg_array)

        # Optional Marker
        for detection in detections:
            marker = Marker()
            marker.header = header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = detection.bbox.center.position.x
            marker.pose.position.y = detection.bbox.center.position.y
            marker.pose.position.z = 0.0
            marker.scale.x = marker.scale.y = marker.scale.z = 10.0
            marker.color.r = 1.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)

        # Show the camera feed with bounding boxes
        cv2.imshow("Camera Feed", color_image)
        cv2.waitKey(1)

    def match_or_create_obstacle(self, x, y, z, cls):
        """Match detected obstacle to existing ones or create new one"""
        current_time = self.get_clock().now()
        
        # Check if any existing obstacle is close enough
        for obs_id, obstacle in self.obstacles.items():
            if obstacle['visible']:
                ox, oy, oz = obstacle['position']
                distance = np.sqrt((x - ox)**2 + (y - oy)**2 + (z - oz)**2)
                if distance < 0.5:  # 50cm threshold
                    return obs_id
        
        # Create new obstacle
        self.obstacle_id_counter += 1
        new_id = self.obstacle_id_counter
        self.obstacles[new_id] = {
            'last_seen': current_time,
            'visible': True,
            'position': (x, y, z),
            'class': cls,
            'bbox': (0, 0, 0, 0)
        }
        return new_id

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()