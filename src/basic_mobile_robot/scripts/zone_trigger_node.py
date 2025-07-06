#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess

class ZoneTriggerNode(Node):
    def __init__(self):
        super().__init__('zone_trigger_node')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.triggered = False  # Ensures script is only run once

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x

        # Map split: x=0 is center. Right side: x > 0
        if x > 0 and not self.triggered:
            self.get_logger().info(f'Robot in right half at x={x:.2f}, triggering script...')
            self.triggered = True
            self.run_bash_script()
        else:
            self.get_logger().info(f'Robot at x={x:.2f}, not triggering.')

    def run_bash_script(self):
        try:
            subprocess.run(['./launch_outdoor.sh'], check=True)
            self.get_logger().info('Bash script executed successfully.')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to run bash script: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ZoneTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
