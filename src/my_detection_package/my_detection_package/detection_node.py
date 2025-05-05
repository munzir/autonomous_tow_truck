import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import torch
import pyrealsense2 as rs
import time
import cv2
import re
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
from collections import defaultdict
from scipy.spatial import KDTree # new

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

        self.bridge = CvBridge()

        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True, autoshape=False)
        # self.model = self.model.to('cuda' if torch.cuda.is_available() else 'cpu').eval()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n')

        # Initialize RealSense pipeline and alignment for depth data
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Camera resolution
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Camera resolution
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        profile = self.pipeline.get_active_profile()
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.width_t = 0.8
        self.height_t = 1.9

        # Track obstacles and their last seen time
        self.obstacles = defaultdict(dict)  # {obstacle_id: {'points': [], 'last_seen': timestamp}}
        self.obstacle_timeout = 1.0  # seconds before removing unseen obstacles
        self.obstacle_id_counter = 0

        # Timer to periodically capture frames
        self.timer = self.create_timer(0.1, self.capture_frame)

    def capture_frame(self):
        # Record the timestamp when the frame is captured
        start_capture_time = time.time()

        # Wait for frames and align the depth frame with the color frame
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        # # Optimized (use poll_for_frames for non-blocking check) # new
        # frames = self.pipeline.poll_for_frames()
        # if frames:
        #     aligned_frames = self.align.process(frames)
        # else:
        #     return  # Skip this iteration if no new frames  
       
        # Get the aligned depth and color frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            self.get_logger().error('Failed to capture depth or color frame')
            return

        # Convert color frame to OpenCV format
        color_image = np.asanyarray(color_frame.get_data())

        # # Perform object detection and draw bounding boxes
        obstacle_detected, annotated_frame, distance = self.detect_obstacle(color_image, depth_frame)

        self.remove_old_obstacles(start_capture_time)

        # Publish updates
        self.publish_combined_pointcloud()

        # Display the camera stream with bounding boxes
        cv2.imshow("Camera Stream", annotated_frame)
        cv2.waitKey(1)  # Wait for 1 ms to update the window

    def detect_obstacle(self, color_image, depth_frame):
        results = self.model(color_image)
        obstacle_detected = False
        current_time = time.time()
        obstacles_detected_in_current_frame = False

        # Reset visibility for all obstacles (will be set to True if detected again)
        for obs_id in self.obstacles:
            self.obstacles[obs_id]['visible'] = False

        # Create header with ROS 2 timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # ROS 2 way to get time
        header.frame_id = "camera_link_optical"  # Match your TF tree

        for *box, conf, cls in results.pred[0]:
            obstacles_detected_in_current_frame = True
            x1, y1, x2, y2 = map(int, box)
            label = f"{results.names[int(cls)]} {conf:.2f}"
            obstacle_detected = True
            no_obstacles_detected = False

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
            # Get or create obstacle ID
            obstacle_id = self.match_or_create_obstacle(X, Y, Z)
            
            # Update obstacle data
            self.obstacles[obstacle_id]['points'] = self.bbox_to_points(x1, y1, x2, y2, depth_frame)
            self.obstacles[obstacle_id]['last_seen'] = current_time
            self.obstacles[obstacle_id]['visible'] = True
            self.obstacles[obstacle_id]['position'] = (X, Y, Z)

        if not obstacles_detected_in_current_frame:
            self.obstacles.clear()
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

        return obstacle_detected, color_image, True
   
    def match_or_create_obstacle(self, x, y, z):
        # Simple obstacle matching based on position (could be improved)
        for obs_id, obs_data in self.obstacles.items():
            if not obs_data['visible']:  # Only match with currently invisible obstacles
                prev_x, prev_y, prev_z = obs_data['position']
                distance = ((x - prev_x)**2 + (y - prev_y)**2 + (z - prev_z)**2)**0.5
                if distance < 0.5:  # Threshold for matching (in meters)
                    return obs_id
        
        # If no match found, create new obstacle
        self.obstacle_id_counter += 1
        return self.obstacle_id_counter
        # if len(self.obstacles) > 0: # new
        #     if not hasattr(self, 'obstacle_kdtree') or self.obstacle_kdtree is None:
        #         positions = [obs['position'] for obs in self.obstacles.values()]
        #         self.obstacle_kdtree = KDTree(positions)
            
        #     dist, idx = self.obstacle_kdtree.query([(x, y, z)], k=1)
        #     if dist[0] < 0.5:  # Matching threshold
        #         return list(self.obstacles.keys())[idx[0]]
        
        # # Create new obstacle
        # self.obstacle_id_counter += 1
        # return self.obstacle_id_counter
    
    def remove_old_obstacles(self, current_time):
        to_remove = []
        for obs_id, obs_data in list(self.obstacles.items()):
            if not obs_data.get('visible', False) and current_time - obs_data['last_seen'] > self.obstacle_timeout:
                to_remove.append(obs_id)

        for obs_id in to_remove:
            self.get_logger().info(f"Removing stale obstacle {obs_id}")
            del self.obstacles[obs_id]

    
    def bbox_to_points(self, x_min, y_min, x_max, y_max, depth_frame):
        point_cloud = []
        depth_image = np.asanyarray(depth_frame.get_data())
        # roi = depth_image[y_min:y_max, x_min:x_max]

        # # Create coordinate grids
        # y_coords, x_coords = np.mgrid[y_min:y_max, x_min:x_max]
        # valid_mask = (roi > 0.1 * 1000) & (roi < 5.0 * 1000)  # Convert meters to mm
        
        # # Vectorized deprojection
        # points = []
        # for y, x in zip(y_coords[valid_mask], x_coords[valid_mask]):
        #     depth = depth_image[y, x] * self.depth_scale
        #     point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [x, y], depth)
        #     points.append(point)
        
        # return points
               
        for y in range(y_min, y_max, 1):  # Sample with stride
            for x in range(x_min, x_max, 1):
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x] * self.depth_scale  # Convert to meters
                    if 0.1 < depth < 5.0:
                        # Convert to 3D coordinates
                        point = rs.rs2_deproject_pixel_to_point(
                            self.depth_intrinsics, 
                            [x, y], 
                            depth
                        )
                        # Add some artificial inflation points around edges
                        if x == x_min or x == x_max-1 or y == y_min or y == y_max-1:
                            for i in range(-3, 4):  # Create points around edges
                                inflated_point = (
                                    point[0] + i*0.05,
                                    point[1] + i*0.05,
                                    point[2]
                                )
                                point_cloud.append(inflated_point)
                        point_cloud.append(point)
        return point_cloud

    def publish_combined_pointcloud(self, points):           
        # Combine all obstacle points
        all_points = []
        for obs_data in self.obstacles.values():
            all_points.extend(obs_data['points'])

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link_optical"
                   
        # else:
        pc_msg = PointCloud2(
            header=header,
            height=1,
            width=len(all_points),
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            is_bigendian=False,
            point_step=12,
            row_step=12 * len(all_points),
            data=np.array(all_points, dtype=np.float32).tobytes(),
            is_dense = True
        )
            
        self.pointcloud_publisher_.publish(pc_msg) 

        # if self.reuse_buffer is None or len(self.reuse_buffer) < len(points): # new
        #     self.reuse_buffer = np.zeros((len(points), 3), dtype=np.float32)
        
        # # Copy data into existing buffer
        # np.copyto(self.reuse_buffer[:len(points)], points)
        
        # pc_msg = PointCloud2(
        #     header=Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_link_optical"),
        #     height=1,
        #     width=len(points),
        #     fields=[...],  # Same as before
        #     is_bigendian=False,
        #     point_step=12,
        #     row_step=12 * len(points),
        #     data=self.reuse_buffer[:len(points)].tobytes(),
        #     is_dense=True
        # )
        # self.pointcloud_publisher_.publish(pc_msg)

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
   
