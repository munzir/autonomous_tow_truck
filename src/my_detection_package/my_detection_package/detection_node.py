import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
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
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
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
        # self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # self.depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        # self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.depth_intrinsics = depth_profile.get_intrinsics()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.width_t = 0.8
        self.height_t = 1.9

        # Track obstacles and their last seen time
        self.obstacles = defaultdict(lambda: {'points': [], 'last_seen': 0, 'visible': False, 'position': (0, 0, 0), 'class': None}) # {obstacle_id: {'points': [], 'last_seen': timestamp}}
        self.obstacle_timeout = 0.2  # seconds before removing unseen obstacles
        self.obstacle_id_counter = 0

        self.width_t = 2.5
        self.height_t = 2.0
        self.frame_lock = Lock()
        # self.timer = self.create_timer(0.1, self.capture_frame)
        

        # Timer to periodically capture frames
        self.timer = self.create_timer(0.1, self.capture_frame)

    def capture_frame(self):
        # Record the timestamp when the frame is captured
        start_capture_time = time.time()

        # # Wait for frames and align the depth frame with the color frame
        # frames = self.pipeline.wait_for_frames()
        # aligned_frames = self.align.process(frames)
       
        # # Get the aligned depth and color frames
        # depth_frame = aligned_frames.get_depth_frame()
        # color_frame = aligned_frames.get_color_frame()

        # if not depth_frame or not color_frame:
        #     self.get_logger().error('Failed to capture depth or color frame')
        #     return
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

        # # # Perform object detection and draw bounding boxes
        # _, annotated_frame, _ = self.detect_obstacle(color_image, depth_frame)

        # self.remove_old_obstacles(start_capture_time)

        # # Publish updates
        # self.publish_combined_pointcloud()

        # # Display the camera stream with bounding boxes
        # cv2.imshow("Camera Stream", annotated_frame)
        # cv2.waitKey(1)  # Wait for 1 ms to update the windo
        with self.frame_lock:
            obstacle_detected, annotated_frame, _ = self.detect_obstacle(color_image, depth_frame)

        self.remove_old_obstacles(start_capture_time)

        if not obstacle_detected and not any(obs_data['visible'] for obs_data in self.obstacles.values()):
            self.get_logger().info("No obstacles detected, publishing empty point cloud")
            self.publish_empty_pointcloud()
        else:
            self.publish_combined_pointcloud()

        cv2.imshow("Camera Stream", annotated_frame)
        cv2.waitKey(1)

    def detect_obstacle(self, color_image, depth_frame):
        results = self.model(color_image)
        obstacle_detected = False
        current_time = time.time()

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_scale = self.depth_scale
        annotated_frame = color_image.copy()
        height, width = depth_image.shape  # (480, 640)

        # Reset visibility for all obstacles (will be set to True if detected again)
        for obs_id in self.obstacles:
            self.obstacles[obs_id]['visible'] = False

        # Create header with ROS 2 timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # ROS 2 way to get time
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

            label = f"{results.names[int(cls)]} {conf:.2f}"
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            depths = []
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    if 0 <= center_x + dx < 640 and 0 <= center_y + dy < 480:
                        d = depth_image[center_y + dy, center_x + dx] * depth_scale
                        if d > 0:
                            depths.append(d)
            Z = np.median(depths) if depths else depth_image[center_y, center_x] * depth_scale

            # Fallback to center pixel if no valid depths
            if np.isnan(Z):
                Z = depth_image[center_y, center_x] * depth_scale
                if np.isnan(Z) or Z < 0.1 or Z > 7.0:
                    self.get_logger().debug(f"Invalid center depth: {Z}m at ({center_x}, {center_y})")
                    continue

            X, Y, Z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], Z)

            # Compute corner depth (for range check)
            if X < 0:
                corner_y, corner_x = y2, x2  # Bottom-right
            else:
                corner_y, corner_x = y1, x1  # Top-left

            corner_depth = depth_image[corner_y, corner_x] * depth_scale

            if np.isnan(corner_depth) or corner_depth < 0.1 or corner_depth > 7.0:
                self.get_logger().debug(f"Invalid corner depth: {corner_depth}m at ({corner_x}, {corner_y})")
                continue

            corner_x_3d, corner_y_3d, corner_z_3d = rs.rs2_deproject_pixel_to_point(
                self.depth_intrinsics, [corner_x, corner_y], corner_depth
            )
            range_within = 0

            # Check if obstacle is within tow truck's range
            if (
                (-self.width_t / 2 - 0.1 < corner_x_3d < self.width_t / 2 + 0.1)
                and (corner_y_3d > -self.height_t / 2 - 0.3)
                and (Z < 9)
            ):
                range_within = 1
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red for in-range
            else:
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green for out-of-range

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
            
            # Generate points for point cloud if within range
            if Z < 7:
                points = self.bbox_to_points(x1, y1, x2, y2, depth_image)
                self.obstacles[obstacle_id]['points'] = points
                self.get_logger().info(
                    f"Obstacle {obstacle_id} at Z={Z:.2f}m with {len(points)} points"
                )
            else:
                self.obstacles[obstacle_id]['points'] = []

            self.obstacles[obstacle_id]['last_seen'] = current_time
            self.obstacles[obstacle_id]['visible'] = True
            self.obstacles[obstacle_id]['position'] = (X, Y, Z)
            self.obstacles[obstacle_id]['class'] = results.names[int(cls)]

        # Clear stale obstacles (older than observation_persistence)
        observation_persistence = 0.2  # Match ObstacleLayer's observation_persistence
        stale_ids = []
        for obs_id, obs in self.obstacles.items():
            if not obs['visible']:
                try:
                    last_seen = obs['last_seen']
                    if isinstance(last_seen, float):
                        self.get_logger().warn(
                            f"Obstacle {obs_id} has float last_seen ({last_seen}), converting to Time"
                        )
                        last_seen = Time(seconds=int(last_seen), nanoseconds=int((last_seen % 1) * 1e9))
                        obs['last_seen'] = last_seen  # Update to Time object
                    if isinstance(last_seen, Time):
                        time_diff = (current_time - last_seen).nanoseconds / 1e9
                        if time_diff > observation_persistence:
                            stale_ids.append(obs_id)
                    else:
                        self.get_logger().warn(f"Invalid last_seen type for obstacle {obs_id}: {type(last_seen)}")
                        stale_ids.append(obs_id)  # Remove invalid entries
                except Exception as e:
                    self.get_logger().error(f"Error processing obstacle {obs_id}: {str(e)}")
                    stale_ids.append(obs_id)  # Remove problematic entries
        
        for obs_id in stale_ids:
            self.get_logger().info(f"Removing stale obstacle {obs_id}")
            del self.obstacles[obs_id]

        if not obstacle_detected:
            self.get_logger().info("No obstacles detected")

        return obstacle_detected, annotated_frame, None
   
    # def match_or_create_obstacle(self, x, y, z):
    #     # Simple obstacle matching based on position (could be improved)
    #     for obs_id, obs_data in self.obstacles.items():
    #         # if not obs_data['visible']:  # Only match with currently invisible obstacles
    #         prev_x, prev_y, prev_z = obs_data['position']
    #         distance = ((x - prev_x)**2 + (y - prev_y)**2 + (z - prev_z)**2)**0.5
    #         if distance < 0.7:  # Threshold for matching (in meters)
    #             return obs_id
        
    #     # If no match found, create new obstacle
    #     self.obstacle_id_counter += 1
    #     return self.obstacle_id_counter

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
        point_cloud = []
        for y in range(y_min, y_max, 2):
            for x in range(x_min, x_max, 2):
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x] * self.depth_scale
                    if not np.isnan(depth) and 0.1 < depth < 5.0:
                        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
                        point_cloud.append(point)
                        if x == x_min or x == x_max-1 or y == y_min or y == y_max-1:
                            for i in [-1, 1]:
                                inflated_point = (
                                    point[0] + i*0.02,
                                    point[1] + i*0.02,
                                    point[2]
                                )
                                point_cloud.append(inflated_point)
        for y in range(0, 480, 20):
            for x in range(0, 640, 20):
                if not (x_min <= x < x_max and y_min <= y < y_max):
                    depth = depth_image[y, x] * self.depth_scale
                    if not np.isnan(depth) and 0.1 < depth < 7.0:
                        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
                        point_cloud.append(point)
        return point_cloud
    
    def remove_old_obstacles(self, current_time):
        to_remove = []
        for obs_id, obs_data in list(self.obstacles.items()):
            if not obs_data['visible'] and current_time - obs_data['last_seen'] > self.obstacle_timeout:
                to_remove.append(obs_id)
        for obs_id in to_remove:
            self.get_logger().info(f"Removing obstacle {obs_id}")
            self.obstacles[obs_id]['points'].clear()
            del self.obstacles[obs_id]
        
        # if to_remove:
        #     self.get_logger().info(f"Removed {len(to_remove)} stale obstacles")

        # self.get_logger().info("After removal:")
        # self.get_logger().info(str(self.obstacles.keys()))
        # self.get_logger().info("\n")

    
    # def bbox_to_points(self, x_min, y_min, x_max, y_max, depth_frame):
    #     point_cloud = []
    #     depth_image = np.asanyarray(depth_frame.get_data())             
    #     for y in range(y_min, y_max, 1):  # Sample with stride
    #         for x in range(x_min, x_max, 1):
    #             if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
    #                 depth = depth_image[y, x] * self.depth_scale  # Convert to meters
    #                 # if np.isnan(depth) or depth <= 0.1 or depth >= 5.0:
    #                 if not np.isnan(depth) and 0.1 < depth < 5.0:
    #                     # Convert to 3D coordinates
    #                     point = rs.rs2_deproject_pixel_to_point(
    #                         self.depth_intrinsics, 
    #                         [x, y], 
    #                         depth
    #                     )
    #                     # Add some artificial inflation points around edges
    #                     if x == x_min or x == x_max-1 or y == y_min or y == y_max-1:
    #                         for i in range(-3, 4):  # Create points around edges
    #                             inflated_point = (
    #                                 point[0] + i*0.05,
    #                                 point[1] + i*0.05,
    #                                 point[2]
    #                             )
    #                             point_cloud.append(inflated_point)
    #                     point_cloud.append(point)
    #     return point_cloud
    def publish_empty_pointcloud(self):
        empty_cloud = PointCloud2(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="camera_link_optical"
            ),
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

    # def publish_combined_pointcloud(self):
    #     points = []

    #     self.get_logger().info("In publish combine pointcloud")

    #     for obs_id,obs_data in self.obstacles.items():
    #         if 'points' in obs_data and obs_data['visible']:
    #             # self.get_logger().info(f"Obstacle {obs_id} contributing {len(obs['points'])} points")
    #             points.extend(obs_data['points'])
    #             self.get_logger().info(f"{obs_id} with {len(obs_data['points'])} points" )
    #             # self.get_logger().info(str(points))

    #      # Filter points: remove invalid or too close points
    #     points = [p for p in points if np.isfinite(p[2]) and p[2] > 0.1]
        
    #     # If no points, publish empty cloud
    #     empty_cloud = PointCloud2(
    #     header=Header(
    #         stamp=self.get_clock().now().to_msg(),
    #         frame_id="camera_link_optical"
    #     ),
    #     height=1,
    #     width=0,
    #     fields=[
    #         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    #     ],
    #     is_bigendian=False,
    #     point_step=12,
    #     row_step=0,
    #     data=bytes(),
    #     is_dense=True
    #     )

    #     if not points:
    #         self.get_logger().info("No visible obstacles, publishing empty point cloud")
    #         self.pointcloud_publisher_.publish(empty_cloud)
    #         return  # Exit after publishing the empty cloud
        
    #     # Build PointCloud2 from points
    #     # cloud_data = []
    #     # for x, y, z in points:
    #     #     cloud_data.extend([x, y, z])
    #     # self.get_logger().info("getting cloud data")

    #     pc_msg = PointCloud2(
    #         header=Header(
    #         stamp=self.get_clock().now().to_msg(),
    #         frame_id="camera_link_optical"
    #     ),
    #         height=1,
    #         width=len(points),
    #         fields=[
    #             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    #             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    #             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    #         ],
    #         is_bigendian=False,
    #         point_step=12,
    #         row_step=12 * len(points),
    #         data=np.array(points, dtype=np.float32).tobytes(),
    #         is_dense = True
    #     )

    #     self.get_logger().info(f"Publishing {len(points)} points")
    #     self.pointcloud_publisher_.publish(pc_msg)

    def publish_combined_pointcloud(self):
        points = []
        visible_obstacles = [obs_id for obs_id, obs_data in self.obstacles.items() if obs_data['visible']]
        for obs_id in visible_obstacles:
            points.extend(self.obstacles[obs_id]['points'])
        
        points = [p for p in points if np.isfinite(p[2]) and p[2] > 0.1]
        
        if not points:
            self.get_logger().info("Publishing empty point cloud")
            pc_msg = PointCloud2(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id="camera_link_optical"
                ),
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
        else:
            self.get_logger().info(f"Publishing {len(points)} points from {len(visible_obstacles)} obstacles")
            pc_msg = PointCloud2(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id="camera_link_optical"
                ),
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
   
