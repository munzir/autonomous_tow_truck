# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
# from visualization_msgs.msg import Marker
# from std_msgs.msg import ColorRGBA
# from cv_bridge import CvBridge
# import numpy as np
# import cv2
# import struct
# from builtin_interfaces.msg import Time

# class ObstacleDetectionNode(Node):
#     def __init__(self):
#         super().__init__('obstacle_detection_node')

#         self.bridge = CvBridge()
#         self.camera_frame = "camera_link"  # Change to your camera's frame
#         self.marker_distance = 0.1  # Distance to extend backward in meters

#         # Publishers
#         self.pc_pub = self.create_publisher(PointCloud2, '/virtual_lane_obstacles', 10)
#         self.marker_pub = self.create_publisher(Marker, '/virtual_no_go_zones', 10)

#         # Subscribers
#         self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
#         self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
#         self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

#         # Variables
#         self.color_image = None
#         self.depth_image = None
#         self.camera_info = None
#         self.marker_id = 0

#     def info_callback(self, msg):
#         if self.camera_info is None:
#             self.camera_info = msg
#             self.get_logger().info("Camera info received")

#     def depth_callback(self, msg):
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

#     def image_callback(self, msg):
#         if self.camera_info is None or self.depth_image is None:
#             return

#         self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         markers = self.detect_lane_markers(self.color_image)
#         points = []

#         self.marker_id = 0  # Reset marker ID for each frame

#         for pt in markers:
#             x, y = int(pt[0]), int(pt[1])

#             if y >= self.depth_image.shape[0] or x >= self.depth_image.shape[1]:
#                 continue

#             depth = float(self.depth_image[y, x]) / 1000.0  # Convert mm to meters

#             if np.isnan(depth) or depth <= 0.1 or depth > 10.0:
#                 continue

#             fx = self.camera_info.k[0]
#             fy = self.camera_info.k[4]
#             cx = self.camera_info.k[2]
#             cy = self.camera_info.k[5]

#             z = depth
#             x3d = (x - cx) * z / fx
#             y3d = (y - cy) * z / fy

#             point_tip = np.array([x3d, y3d, z])
#             offset = np.array([0.0, self.marker_distance, 0.0])
#             point_back = point_tip + offset
#             points.append(point_back)

#             # Publish a red translucent cube for visualization
#             marker = Marker()
#             marker.header.frame_id = self.camera_frame
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "no_go_zone"
#             marker.id = self.marker_id
#             marker.type = Marker.CUBE
#             marker.action = Marker.ADD
#             marker.scale.x = 0.2  # Width
#             marker.scale.y = self.marker_distance  # Depth
#             marker.scale.z = 0.1  # Height
#             marker.pose.position.x = point_tip[0]
#             marker.pose.position.y = point_tip[1] + marker.scale.y / 2.0
#             marker.pose.position.z = point_tip[2]
#             marker.pose.orientation.x = 0.0
#             marker.pose.orientation.y = 0.0
#             marker.pose.orientation.z = 0.0
#             marker.pose.orientation.w = 1.0
#             marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
#             self.marker_pub.publish(marker)
#             self.marker_id += 1

#         self.publish_pointcloud(points)

#     def detect_lane_markers(self, image):
#         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         lower = np.array([20, 100, 100])
#         upper = np.array([30, 255, 255])
#         mask = cv2.inRange(hsv, lower, upper)
#         contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#         marker_points = []
#         for cnt in contours:
#             M = cv2.moments(cnt)
#             if M['m00'] != 0:
#                 cx = int(M['m10'] / M['m00'])
#                 cy = int(M['m01'] / M['m00'])
#                 marker_points.append((cx, cy))
#         return marker_points

#     def publish_pointcloud(self, points):
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#         ]

#         pc_data = []
#         for p in points:
#             pc_data.append(struct.pack('fff', *p))

#         cloud_msg = PointCloud2()
#         cloud_msg.header.stamp = self.get_clock().now().to_msg()
#         cloud_msg.header.frame_id = self.camera_frame
#         cloud_msg.height = 1
#         cloud_msg.width = len(points)
#         cloud_msg.fields = fields
#         cloud_msg.is_bigendian = False
#         cloud_msg.point_step = 12
#         cloud_msg.row_step = cloud_msg.point_step * len(points)
#         cloud_msg.is_dense = True
#         cloud_msg.data = b''.join(pc_data)

#         self.pc_pub.publish(cloud_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleDetectionNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker
import numpy as np
import pyrealsense2 as rs
import time
import cv2
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped

class SafetyMarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('safety_marker_detection_node')
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.marker_publisher_ = self.create_publisher(Marker, '/safety_marker', qos_profile)
        self.image_publisher_ = self.create_publisher(Image, '/marker_detection_image', qos_profile)
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, '/safety_marker_points', qos_profile)
        self.bridge = CvBridge()

        # Initialize RealSense pipeline (now including depth)
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # Get depth scale and intrinsics
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        # Timer to periodically capture frames
        self.timer = self.create_timer(0.1, self.capture_frame)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_camera_tf()

        # Marker detection parameters
        self.min_line_length = 20  # Minimum line length to be considered a marker
        self.max_line_gap = 5     # Maximum gap between line segments
        self.line_threshold = 50  # Accumulator threshold for line detection
        self.wall_height = 1.5    # Estimated height of walls in meters
        self.wall_thickness = 0.1 # Thickness to give the wall points

    def publish_camera_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "camera_link_optical"
        transform.transform.translation.x = 0.305
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.08
        transform.transform.rotation.x = -0.707
        transform.transform.rotation.w = 0.707
        self.tf_broadcaster.sendTransform(transform)

    def define_trapezoid_roi(self, image):
        H, W = image.shape[:2]
        bottom_left = (int(0.01 * W), H)
        bottom_right = (int(0.99 * W), H)
        top_left = (int(0.35 * W), int(0.6 * H))
        top_right = (int(0.65 * W), int(0.6 * H))
        
        mask = np.zeros_like(image, dtype=np.uint8)
        trapezoid = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        cv2.fillPoly(mask, [trapezoid], (255, 255, 255))
        return mask

    def capture_frame(self):
        try:
            # Wait for frames and align them
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                self.get_logger().warn('No color or depth frame available')
                return

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Detect markers and get their 3D points
            marker_detected, annotated_frame, marker_lines = self.detect_markers(color_image, depth_image)

            # Publish visualization image
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_link_optical"
            self.image_publisher_.publish(ros_image)

            # Publish point cloud if markers detected
            if marker_detected:
                pc_msg = self.create_pointcloud(marker_lines)
                self.pointcloud_publisher_.publish(pc_msg)
                
                marker = self.create_marker(len(marker_lines))
                self.marker_publisher_.publish(marker)
                self.get_logger().info(f"Detected {len(marker_lines)} safety markers")

            # Display the result
            cv2.imshow("Safety Marker Detection", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in capture_frame: {str(e)}")

    def detect_markers(self, color_image, depth_image):
        # Apply trapezoid ROI
        mask = self.define_trapezoid_roi(color_image)
        roi_frame = cv2.bitwise_and(color_image, mask)

        # Convert to grayscale and detect edges
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 75, 150)

        # Detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, self.line_threshold, 
                              minLineLength=self.min_line_length, 
                              maxLineGap=self.max_line_gap)

        marker_detected = False
        marker_lines = []
        annotated_frame = color_image.copy()

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                
                # Filter for near-vertical lines (typical of safety markers)
                if (40 < abs(angle) < 80) or (100 < abs(angle) < 135):
                    cv2.line(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    marker_detected = True
                    
                    # Store line endpoints with depth information
                    marker_lines.append({
                        'p1': (x1, y1),
                        'p2': (x2, y2),
                        'depth1': depth_image[y1, x1] * self.depth_scale,
                        'depth2': depth_image[y2, x2] * self.depth_scale
                    })

        return marker_detected, annotated_frame, marker_lines

    def create_pointcloud(self, marker_lines):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link_optical"

        points = []
        for x in np.linspace(-0.5, 0.5, 10):
            for y in np.linspace(-0.5, 0.5, 10):
                points.append([x, y, 2.0])
            
        for line in marker_lines:
            x1, y1 = line['p1']
            x2, y2 = line['p2']
            depth1 = line['depth1']
            depth2 = line['depth2']
            
            # Convert endpoints to 3D points
            point1 = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x1, y1], depth1)
            point2 = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x2, y2], depth2)
            
            # Create vertical wall by adding points at different heights
            for height in np.linspace(0, self.wall_height, 5):
                # Add thickness to the wall by offsetting points slightly
                for offset in np.linspace(-self.wall_thickness/2, self.wall_thickness/2, 3):
                    points.append([point1[0] + offset, point1[1] + offset, height])
                    points.append([point2[0] + offset, point2[1] + offset, height])

        # Add debug print
        self.get_logger().info(f"Generated {len(points)} points for point cloud")
        
        if not points:
            self.get_logger().warn("No points generated for point cloud!")
            return None

        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pc_data = np.array(points, dtype=np.float32).flatten()
        
        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12  # 3 floats (x,y,z) * 4 bytes each
        pc_msg.row_step = pc_msg.point_step * len(points)
        pc_msg.data = pc_data.tobytes()
        pc_msg.is_dense = True

        return pc_msg

    def create_marker(self, count):
        marker = Marker()
        marker.header.frame_id = "camera_link_optical"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = f"{count} Safety Markers"
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMarkerDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()