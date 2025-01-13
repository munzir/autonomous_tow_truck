#! /usr/bin/env python3

import csv
import time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from enum import Enum
import yaml

def save_path_to_file(path, file_path):
    path_dict = {
        'header': {
            'frame_id': 'map'  # Ensure the header.frame_id is set for the entire path
        },
        'poses': []
    }
    for pose_stamped in path.poses:
        pose_data = {
            'header': {
                'frame_id': 'map'  # Ensure each pose's header.frame_id is set
            },
            'position': {
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'z': pose_stamped.pose.position.z
            },
            'orientation': {
                'x': pose_stamped.pose.orientation.x,
                'y': pose_stamped.pose.orientation.y,
                'z': pose_stamped.pose.orientation.z,
                'w': pose_stamped.pose.orientation.w
            }
        }
        path_dict['poses'].append(pose_data)

    try:
        with open(file_path, 'w') as file:
            yaml.dump(path_dict, file)
        print(f'Path saved to {file_path}')
    except Exception as e:
        print(f'Failed to save path: {e}')

class TaskResult(Enum):
    UKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Action client for ComputePathThroughPoses
        self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')
        
        # QoS settings for subscription
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Subscriber to the initial pose (AMCL)
        self.initial_pose_received = False

    def load_waypoints(self, file_path):
        waypoints = []
        try:
            with open(file_path, mode='r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if len(row) == 4:
                        x, y, w, z = map(float, row)
                        pose = PoseStamped()
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        pose.pose.orientation.w = w
                        pose.pose.orientation.z = z
                        pose.header.frame_id = 'map'
                        waypoints.append(pose)
            self.info(f'Loaded {len(waypoints)} waypoints from {file_path}')
        except Exception as e:
            self.error(f'Failed to load waypoints: {e}')
        
        return waypoints

    def compute_path_through_poses(self, start, poses):
        # Sends a `ComputePathThroughPoses` action request
        self.debug("Waiting for 'ComputePathThroughPoses' action server")
        while not self.compute_path_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathThroughPoses' action server not available, waiting...")

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = poses
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_through_poses_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.error('Path request was rejected!')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {status}')
            return None

        self.info('Path successfully computed!')
        return result_future.result().result.path

    def run(self):
        # Wait for the Nav2 stack and action servers to be active
        self.wait_until_nav2_active()

        # Load waypoints from CSV
        waypoints = self.load_waypoints('/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints.csv')

        if not waypoints:
            self.error('No waypoints loaded. Exiting.')
            return

        # Assume the initial pose is the starting point
        start_pose = waypoints[0]  # Taking the first waypoint as the starting point

        # Compute the path through all the waypoints
        path = self.compute_path_through_poses(start=start_pose, poses=waypoints)

        if path:
            self.info(f'Computed path with {len(path.poses)} poses.')
            path_file = '/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/saved_path.yaml'
            save_path_to_file(path, path_file)
        else:
            self.error('Failed to compute the path.')

    def wait_until_nav2_active(self):
        # Ensure that the Nav2 stack is active and ready
        self.info('Waiting for Nav2 stack to be active...')
        # Here, you might want to add specific checks for the Nav2 nodes like 'amcl', 'bt_navigator', etc.
        time.sleep(2)  # For now, just a simple sleep

    def info(self, msg):
        self.get_logger().info(msg)

    def warn(self, msg):
        self.get_logger().warn(msg)

    def error(self, msg):
        self.get_logger().error(msg)

    def debug(self, msg):
        self.get_logger().debug(msg)


def main(args=None):
    rclpy.init(args=args)

    waypoint_navigator = WaypointNavigator()
    waypoint_navigator.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

