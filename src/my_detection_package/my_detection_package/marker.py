import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import numpy as np
import cv2
import struct
from builtin_interfaces.msg import Time

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        self.bridge = CvBridge()
        self.camera_frame = "camera_link"  # Change to your camera's frame
        self.marker_distance = 0.1  # Distance to extend backward in meters

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/virtual_lane_obstacles', 10)
        self.marker_pub = self.create_publisher(Marker, '/virtual_no_go_zones', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        # Variables
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.marker_id = 0

    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("Camera info received")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            return

        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        markers = self.detect_lane_markers(self.color_image)
        points = []

        self.marker_id = 0  # Reset marker ID for each frame

        for pt in markers:
            x, y = int(pt[0]), int(pt[1])

            if y >= self.depth_image.shape[0] or x >= self.depth_image.shape[1]:
                continue

            depth = float(self.depth_image[y, x]) / 1000.0  # Convert mm to meters

            if np.isnan(depth) or depth <= 0.1 or depth > 10.0:
                continue

            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            z = depth
            x3d = (x - cx) * z / fx
            y3d = (y - cy) * z / fy

            point_tip = np.array([x3d, y3d, z])
            offset = np.array([0.0, self.marker_distance, 0.0])
            point_back = point_tip + offset
            points.append(point_back)

            # Publish a red translucent cube for visualization
            marker = Marker()
            marker.header.frame_id = self.camera_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "no_go_zone"
            marker.id = self.marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = 0.2  # Width
            marker.scale.y = self.marker_distance  # Depth
            marker.scale.z = 0.1  # Height
            marker.pose.position.x = point_tip[0]
            marker.pose.position.y = point_tip[1] + marker.scale.y / 2.0
            marker.pose.position.z = point_tip[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            self.marker_pub.publish(marker)
            self.marker_id += 1

        self.publish_pointcloud(points)

    def detect_lane_markers(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([20, 100, 100])
        upper = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        marker_points = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                marker_points.append((cx, cy))
        return marker_points

    def publish_pointcloud(self, points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        pc_data = []
        for p in points:
            pc_data.append(struct.pack('fff', *p))

        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = self.camera_frame
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(pc_data)

        self.pc_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
