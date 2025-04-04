from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
import numpy as np
import re

class ObstacleGridNode(Node):
    def __init__(self):
        super().__init__('obstacle_grid_node')

        self.sub = self.create_subscription(
            String,
            '/yolo_detections',
            self.detection_callback,
            10
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            '/obstacle_grid',
            10
        )

        self.grid_size = 100  # 100x100 grid
        self.grid_res = 0.1   # Each cell = 10cm

    def detection_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

        # Parse box center from the string
        match = re.search(r"Box: \((\d+), (\d+), (\d+), (\d+)\)", msg.data)
        if not match:
            self.get_logger().warn("Box not found in string.")
            return

        x_min, y_min, x_max, y_max = map(int, match.groups())
        center_x = (x_min + x_max) // 2
        center_y = (y_min + y_max) // 2

        # Fake grid placement: just map center_x/y to grid coordinates for demo
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        grid_x = center_x % self.grid_size
        grid_y = center_y % self.grid_size
        grid[grid_y, grid_x] = 100

        self.publish_occupancy_grid(grid)

    def publish_occupancy_grid(self, grid):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid_res
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        msg.data = grid.flatten().tolist()

        self.pub.publish(msg)
        self.get_logger().info("Published occupancy grid from YOLO detections.")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
