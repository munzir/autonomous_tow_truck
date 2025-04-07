from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.node import Node
import numpy as np
import re
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point32
import std_msgs

class ObstaclePointCloudNode(Node):
    def __init__(self):
        super().__init__('obstacle_pointcloud_node')

        self.sub = self.create_subscription(
            String,
            '/yolo_detections',
            self.detection_callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/obstacle_pointcloud',
            10
        )

        self.grid_size = 100  # Not used for PointCloud2, but you can keep it if you need grid-related data
        self.grid_res = 0.1   # Resolution for grid (not needed for PointCloud2, but kept for reference)

    def detection_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

        # Parse box center from the string
        match = re.search(r"Box: \((\d+), (\d+), (\d+), (\d+)\)", msg.data)
        if not match:
            self.get_logger().warn("Box not found in string.")
            return

        x_min, y_min, x_max, y_max = map(int, match.groups())
        
        # Calculate the center of the bounding box
        center_x = (x_min + x_max) // 2
        center_y = (y_min + y_max) // 2

        # For simplicity, assume the depth (Z) to be constant or derived from the depth camera
        # In a real application, you'd get the Z value from a depth sensor (like RealSense)
        depth_z = 1.0  # Placeholder for actual depth value

        self.get_logger().info(f"Center X: {center_x}, Center Y: {center_y}, Depth Z: {depth_z}")

        # Create the 3D point (you can scale these based on your grid resolution)
        point = Point32()
        point.x = center_x * self.grid_res  # Convert pixel to meters (scaled by grid resolution)
        point.y = center_y * self.grid_res
        point.z = depth_z  # The depth (z-coordinate) in meters

        # Publish a PointCloud2 message
        self.publish_pointcloud(point)

    def publish_pointcloud(self, points):
        # Ensure points is a list (even if single point)
        if isinstance(points, Point32):
            points = [points]  # Convert single Point32 to a list of one Point32

        # Convert Point32 objects to tuples of (x, y, z)
        point_tuples = [(p.x, p.y, p.z) for p in points]

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # Create the point cloud using the coordinate tuples
        cloud_data = pc2.create_cloud_xyz32(header, point_tuples)

        # Publish the point cloud message
        self.pub.publish(cloud_data)
        self.get_logger().info("Published PointCloud2 with detected obstacles.")
def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()