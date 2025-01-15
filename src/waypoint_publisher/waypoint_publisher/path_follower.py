import rclpy
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import yaml


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Initialize action client for FollowPath
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.get_logger().info("Waiting for 'FollowPath' action server...")
        self.wait_for_action_server()

        # Load the path from the YAML file
        self.path = self.load_path('/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/saved_path.yaml')

        # Call follow path method
        self.follow_path(self.path)

    def wait_for_action_server(self):
        """Wait for the FollowPath action server to be available."""
        while not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'FollowPath' action server not available, waiting...")

    def load_path(self, file_path):
        """Load the path from a YAML file."""
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            path = Path()  # Create Path message
            path.header.frame_id = 'map'
            for pose_data in data['poses']:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = pose_data['position']['x']
                pose.pose.position.y = pose_data['position']['y']
                pose.pose.position.z = pose_data['position']['z']

                pose.pose.orientation.w = pose_data['orientation']['w']
                pose.pose.orientation.x = pose_data['orientation']['x']
                pose.pose.orientation.y = pose_data['orientation']['y']
                pose.pose.orientation.z = pose_data['orientation']['z']

                path.poses.append(pose)
            return path

    def follow_path(self, path):
        """Send a FollowPath action request."""
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        self.get_logger().info('Executing path...')
        send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Follow path was rejected!')
            return False

        self.get_logger().info('Follow path accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        # Checking the status of the action result
        if result.status == 3:  # STATUS_SUCCEEDED (3) in ROS2
            self.get_logger().info('Path successfully followed!')
        else:
            self.get_logger().error(f'Failed to follow path, status: {result.status}')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

