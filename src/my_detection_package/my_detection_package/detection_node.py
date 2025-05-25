import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import torch
import pyrealsense2 as rs
import time
import cv2
from threading import Lock
from collections import defaultdict

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        qos_profile = rclpy.qos.QoSProfile(
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,# self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
                depth=1
            )
        self.pointcloud_publisher_ = self.create_publisher(
            PointCloud2, 
            '/yolo_detections',  # Standard RealSense topic
            qos_profile
        )        

        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/my_detection_package/my_detection_package/best.pt')

        # Initialize RealSense pipeline and alignment for depth data
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Camera resolution
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Camera resolution
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth)) 
        self.depth_intrinsics = depth_profile.get_intrinsics()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        self.width_t = 0.8
        self.height_t = 1.9

        # Track obstacles and their last seen time
        self.obstacles = defaultdict(lambda: {
            'points': [], 
            'last_seen': Time(seconds=0), 
            'visible': False, 
            'position': (0, 0, 0), 
            'class': None
            }) # {obstacle_id: {'points': [], 'last_seen': timestamp}}
        
        self.obstacle_timeout = 0.2  # seconds before removing unseen obstacles
        self.obstacle_id_counter = 0

        self.frame_lock = Lock()       

        # Timer to periodically capture frames
        self.timer = self.create_timer(0.1, self.capture_frame)

    def capture_frame(self):
        with self.frame_lock:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            self.get_logger().error('Failed to capture depth or color frame')
            return

        # Convert color frame to OpenCV format
        color_image = np.asarray(color_frame.get_data())
        obstacle_detected, annotated_frame, header = self.detect_obstacle(color_image, depth_frame)
        
        current_time = self.get_clock().now()
        self.remove_old_obstacles(current_time)
        self.publish_combined_pointcloud(header)

        cv2.imshow("Camera Stream", annotated_frame)
        cv2.waitKey(1)

    def detect_obstacle(self, color_image, depth_frame):
        results = self.model(color_image)
        obstacle_detected = False
        current_time = self.get_clock().now()

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_scale = self.depth_scale
        annotated_frame = color_image.copy()
        height, width = depth_image.shape  # (480, 640)

        # Reset visibility for all obstacles (will be set to True if detected again)
        for obs_id in self.obstacles:
            self.obstacles[obs_id]['visible'] = False

        # Create header with ROS 2 timestamp
        header = Header()
        header.stamp = current_time.to_msg()  # ROS 2 way to get time
        header.frame_id = "camera_link_optical"  # Match your TF tree

        for *box, conf, cls in results.pred[0]:
            obstacle_detected = True
            x1, y1, x2, y2 = map(int, box)
            x1 = max(0, min(x1, width - 1))  # 0 to 639
            y1 = max(0, min(y1, height - 1))  # 0 to 479
            x2 = max(0, min(x2, width - 1))
            y2 = max(0, min(y2, height - 1))

            # Skip invalid boxes
            if x1 >= x2 or y1 >= y2:
                self.get_logger().debug(f"Invalid box skipped: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                continue

            # Skip detections near frame bottom (likely ground)
            if y2 > 460:
                self.get_logger().debug(f"Skipping ground detection: y2={y2}")
                continue

            label = f"{results.names[int(cls)]} {conf:.2f}"
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Compute depth from 5x5 region
            depths = []
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    px = center_x + dx
                    py = center_y + dy
                    if 0 <= px < width and 0 <= py < height:
                        d = depth_image[py, px] * depth_scale
                        if not np.isnan(d) and 0.1 < d < 10.0:
                            depths.append(d)
            Z = np.median(depths) if depths else depth_image[center_y, center_x] * depth_scale

            if np.isnan(Z) or Z < 0.1 or Z > 10.0:
                self.get_logger().debug(f"Invalid center depth: {Z}m at ({center_x}, {center_y})")
                continue

            # Skip points too close to camera (possible tow truck body)
            if Z < 0.2:
                self.get_logger().debug(f"Point too close to camera: Z={Z}m at ({center_x}, {center_y})")
                continue

            X, Y, Z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], Z)

            # Skip ground points (Y too low)
            if Y < -0.3:
                self.get_logger().debug(f"Ground point skipped: Y={Y:.2f}m at ({center_x}, {center_y})")
                continue

            # Log detection details
            self.get_logger().info(f"Detection: class={results.names[int(cls)]}, box=({x1},{y1},{x2},{y2}), 3D=({X:.2f},{Y:.2f},{Z:.2f})")

            # Corner depth for range check
            corner_y, corner_x = (y2, x2) if X < 0 else (y1, x1)
            corner_depth = depth_image[corner_y, corner_x] * depth_scale
            if np.isnan(corner_depth) or corner_depth < 0.1 or corner_depth > 10.0:
                self.get_logger().debug(f"Invalid corner depth: {corner_depth}m at ({corner_x}, {corner_y})")
                continue
            corner_x_3d, corner_y_3d, corner_z_3d = rs.rs2_deproject_pixel_to_point(
                self.depth_intrinsics, [corner_x, corner_y], corner_depth
            )

            # Range check
            range_within = 0
            if (
                (-self.width_t / 2 - 0.1 < corner_x_3d < self.width_t / 2 + 0.1)
                and (corner_y_3d > -self.height_t / 2 - 0.3)
                and (Z < 10)
            ):
                range_within = 1
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            else:
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            obstacle_id = self.match_or_create_obstacle(X, Y, Z, results.names[int(cls)])
            cv2.putText(
                annotated_frame,
                f"ID:{obstacle_id} Z:{Z:.2f}m",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            # Generate obstacle points
            points = self.bbox_to_points(x1, y1, x2, y2, depth_image)
            self.obstacles[obstacle_id]['points'] = points
            self.get_logger().info(
                f"Obstacle {obstacle_id} at Z={Z:.2f}m with {len(points)} points"
            )

            self.obstacles[obstacle_id]['last_seen'] = current_time
            self.obstacles[obstacle_id]['visible'] = True
            self.obstacles[obstacle_id]['position'] = (X, Y, Z)
            self.obstacles[obstacle_id]['class'] = results.names[int(cls)]

        # # Generate and store clearing points
        # clearing_points = self.generate_clearing_pointcloud(depth_image)
        # self.obstacles['clearing'] = {
        #     'points': clearing_points,
        #     'last_seen': current_time,
        #     'visible': True,
        #     'position': (0, 0, 0),
        #     'class': 'clearing'
        # }

        return obstacle_detected, annotated_frame, header

    def generate_clearing_pointcloud(self, depth_image):
        points = []
        height, width = depth_image.shape  # (480, 640)
        step = 10  # ~4000 points
        depth_scale = self.depth_scale

        for y in range(10, height-10, step):  # Exclude edge pixels
            for x in range(10, width-10, step):
                depth = depth_image[y, x] * depth_scale
                if not np.isnan(depth) and 0.1 < depth < 9.5:
                    point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
                    # Log corner points for debugging
                    if x <= 20 or y <= 20:
                        self.get_logger().debug(f"Corner point: x={x}, y={y}, depth={depth:.2f}, 3D=({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")
                    points.append(point)

        self.get_logger().info(f"Generated {len(points)} clearing points")
        return points

    def match_or_create_obstacle(self, x, y, z, class_label):
        for obs_id, obs_data in self.obstacles.items():
            prev_x, prev_y, prev_z = obs_data['position']
            prev_class = obs_data.get('class', None)
            distance = ((x - prev_x)**2 + (y - prev_y)**2 + (z - prev_z)**2)**0.5
            if distance < 0.7 and prev_class == class_label:
                self.get_logger().info(f"Matched obstacle ID {obs_id} at ({x:.2f}, {y:.2f}, {z:.2f})")
                return obs_id
        self.obstacle_id_counter += 1
        self.get_logger().info(f"Created new obstacle ID {self.obstacle_id_counter} at ({x:.2f}, {y:.2f}, {z:.2f})")
        return self.obstacle_id_counter
    
    def bbox_to_points(self, x_min, y_min, x_max, y_max, depth_image):
        points = []
        depth_scale = self.depth_scale
        for y in range(y_min, y_max, 2):
            for x in range(x_min, x_max, 2):
                if x < 20 or x > 620 or y < 20 or y > 460:
                    self.get_logger().debug(f"Skipping edge pixel: x={x}, y={y}")
                    continue
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x] * depth_scale
                    if not np.isnan(depth) and 0.2 < depth < 9.5:
                        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
                        # Filter ground points
                        if point[1] < -0.3:
                            self.get_logger().debug(f"Ground obstacle point skipped: Y={point[1]:.2f}, 3D=({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")
                            continue
                        points.append(point)
                        # Log edge points
                        if point[2] > 9.0:
                            self.get_logger().debug(f"Edge point: 3D=({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")
                        if x == x_min or x == x_max-1 or y == y_min or y == y_max-1:
                            for i in [-1, 1]:
                                inflated_point = (
                                    point[0] + i*0.02,
                                    point[1] + i*0.02,
                                    point[2]
                                )
                                 # Filter ground and edge for inflated points
                                if inflated_point[1] < -0.3 or inflated_point[2] > 9.5:
                                    continue
                                points.append(inflated_point)
        self.get_logger().debug(f"Generated {len(points)} obstacle points for box ({x_min},{y_min},{x_max},{y_max})")
        return points
    
    def remove_old_obstacles(self, current_time):
        to_remove = []
        for obs_id, obs in self.obstacles.items():
            if obs_id == 'clearing':
                continue
            last_seen = obs['last_seen']
            if not obs['visible'] and isinstance(last_seen, Time):
                time_diff = (current_time - last_seen).nanoseconds / 1e9
                if time_diff > self.obstacle_timeout:
                    to_remove.append(obs_id)
        for obs_id in to_remove:
            self.get_logger().info(f"Removing stale obstacle {obs_id}")
            del self.obstacles[obs_id]
        
    def publish_empty_pointcloud(self, header):
        empty_cloud = PointCloud2(
            header=header,
            height=1,
            width=0,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            is_bigendian=False,
            point_step=12,
            row_step=0,
            data=bytes(),
            is_dense=True
        )
        self.pointcloud_publisher_.publish(empty_cloud)
        self.get_logger().info("Published empty point cloud")


    def publish_combined_pointcloud(self, header):
        points = []
        visible_obstacles = [obs_id for obs_id, obs_data in self.obstacles.items() if obs_data['visible']]
        for obs_id in visible_obstacles:
            points.extend(self.obstacles[obs_id]['points'])
        
        points = [p for p in points if np.isfinite(p[2]) and 0.2 < p[2] < 9.5]
        
        # Log point details
        if points:
            sample_points = points[:5]  # Sample up to 5 points
            for p in sample_points:
                self.get_logger().debug(f"Published point: 3D=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
            # Log edge points
            edge_points = [p for p in points if p[2] > 9.0]
            if edge_points:
                self.get_logger().warn(f"Found {len(edge_points)} points near max range: sample={edge_points[:3]}")

        if not points:
            self.get_logger().info("No valid points. Publishing empty point cloud")
            self.publish_empty_pointcloud(header)
            return
        
        pc_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            is_bigendian=False,
            point_step=12,
            row_step=12 * len(points),
            data=np.array(points, dtype=np.float32).tobytes(),
            is_dense=True
        )
        self.pointcloud_publisher_.publish(pc_msg)
        self.get_logger().info(f"Publishing {len(points)} points from {len(visible_obstacles)} obstacles")

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
   
