import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import os

class StaticTransformPublisher(Node):

    def __init__(self):
        super().__init__('static_transform_publisher')

        # Declare the publisher for static transform
        self.publisher = tf2_ros.StaticTransformBroadcaster(self)

        # Load the pose from the CSV file
        self.pose = self.read_pose_from_csv('/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/initial_pose.csv')

        # Create and publish the static transform
        self.publish_static_transform()

    def read_pose_from_csv(self, file_path):
        # Check if the file exists
        if not os.path.exists(file_path):
            self.get_logger().error(f"File {file_path} does not exist!")
            rclpy.shutdown()
            return None
        
        # Read the first line from the CSV and extract the pose values
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            row = next(reader)  # Read the first line

            # Ensure we have enough values
            if len(row) < 4:
                self.get_logger().error("CSV file does not have enough values.")
                rclpy.shutdown()
                return None

            # Extract position (x, y) and quaternion (w, z)
            x = float(row[0])
            y = float(row[1])
            w = float(row[2])
            z = float(row[3])

            return (x, y, w, z)

    def publish_static_transform(self):
        if self.pose is None:
            return

        x, y, w, z = self.pose

        # Create the static transform message
        transform = TransformStamped()

        # Set the header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'

        # Set the translation
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        # Set the rotation (quaternion)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w

        # Publish the static transform
        self.publisher.sendTransform(transform)
        self.get_logger().info('Published static transform from map to odom.')

def main(args=None):
    rclpy.init(args=args)

    static_transform_publisher = StaticTransformPublisher()

    # Spin the node
    rclpy.spin(static_transform_publisher)

    # Clean up after the spin
    static_transform_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

