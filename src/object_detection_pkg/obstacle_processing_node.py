import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import numpy as np
from cv_bridge import CvBridge
from collections import defaultdict
import time  # Needed for tracking obstacle timestamps

class ObstacleProcessingNode(Node):
    def __init__(self):
        super().__init__('obstacle_processing_node')

        # Declare sim time parameter
        self.declare_parameter('use_sim_time', True)

        # Read the sim time parameter
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.get_logger().info(f"use_sim_time set to: {self.use_sim_time}")

        # Set the parameter for ROS 2 clock sync
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, self.use_sim_time)])

        # Subscribers
        self.detection_sub = self.create_subscription(Detection2DArray, '/yolo_detections', self.detection_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        # Publisher
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/obstacle_grid', 10)

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Depth image and camera parameters
        self.latest_depth = None
        self.fx, self.fy, self.cx, self.cy = None, None, None, None

        # CV Bridge
        self.bridge = CvBridge()

        # Store timestamps of detected obstacles
        self.obstacle_timestamps = defaultdict(float)
        self.decay_time = 5.0  # Remove obstacles after 5 seconds

    def camera_info_callback(self, msg):
        """Extract camera intrinsic parameters."""
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.get_logger().info("Camera intrinsics received.")

    def depth_callback(self, msg):
        """Convert depth image to a NumPy array."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def detection_callback(self, msg):
        """Process YOLO detections and update occupancy grid."""
        if self.latest_depth is None or self.fx is None:
            self.get_logger().warn("Waiting for depth image and camera parameters...")
            return

        grid_size = 100
        grid_res = 0.1
        grid = np.full((grid_size, grid_size), -1, dtype=np.int8)  # Initialize with unknown values
        current_time = time.time()

        for detection in msg.detections:
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)

            # Ensure within bounds
            if 0 <= center_x < self.latest_depth.shape[1] and 0 <= center_y < self.latest_depth.shape[0]:
                depth = float(self.latest_depth[center_y, center_x])

                # Validate depth
                if np.isnan(depth) or depth <= 0 or depth > 10:
                    continue

                # Convert to 3D Camera Coordinates
                X = (center_x - self.cx) * depth / self.fx
                Y = (center_y - self.cy) * depth / self.fy

                # Transform to Map Frame
                try:
                    trans = self.tf_buffer.lookup_transform("map", "camera_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
                    X_map = trans.transform.translation.x + X
                    Y_map = trans.transform.translation.y + Y
                except Exception as e:
                    self.get_logger().warn(f"TF lookup failed: {e}")
                    continue

                # Convert to Grid Index
                grid_x = int((X_map + 5) / grid_res)  # Assuming grid origin (-5, -5)
                grid_y = int((Y_map + 5) / grid_res)

                # Ensure valid grid coordinates
                if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                    grid[grid_y, grid_x] = 100  # Mark as occupied
                    self.obstacle_timestamps[(grid_x, grid_y)] = current_time  # Update timestamp

        # Apply Decay: Remove old obstacles
        self.apply_decay(grid, current_time)

        self.publish_occupancy_grid(grid)

    def apply_decay(self, grid, current_time):
        """Remove old obstacles from the grid after decay_time."""
        for (grid_x, grid_y), timestamp in list(self.obstacle_timestamps.items()):
            if current_time - timestamp > self.decay_time:
                grid[grid_y, grid_x] = 0  # Mark as free
                del self.obstacle_timestamps[(grid_x, grid_y)]  # Remove from tracking
                self.get_logger().info(f"Cleared obstacle at ({grid_x}, {grid_y})")

    def publish_occupancy_grid(self, grid):
        """Publish occupancy grid to update costmap."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.1
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        msg.data = grid.flatten().tolist()

        self.occupancy_pub.publish(msg)
        self.get_logger().info("Published updated occupancy grid.")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
