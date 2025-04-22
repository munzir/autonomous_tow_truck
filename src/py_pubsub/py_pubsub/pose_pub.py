import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # For publishing Odometry messages
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile  # For setting Quality of Service
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')
        qos_profile = QoSProfile(depth=10)

        # Subscribers for x, y, theta, and velocity data
        self.subscription_v = self.create_subscription(
            Float32,
            'v',
            self.v_callback,
            qos_profile
        )
        self.subscription_x = self.create_subscription(
            Float32,
            'x',
            self.x_callback,
            qos_profile
        )
        self.subscription_y = self.create_subscription(
            Float32,
            'y',
            self.y_callback,
            qos_profile
        )
        self.subscription_theta = self.create_subscription(
            Float32,
            'theta',
            self.theta_callback,
            qos_profile
        )

        # Publisher for the odometry data
        self.odom_publisher = self.create_publisher(Odometry, 'wheel/odometry', qos_profile)

        # Transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize variables to store the x, y, theta, and velocity values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0  # Initialize velocity

    def v_callback(self, msg):
        self.v = msg.data

    def x_callback(self, msg):
        self.x = msg.data
        self.publish_odometry()

    def y_callback(self, msg):
        self.y = msg.data
        self.publish_odometry()

    def theta_callback(self, msg):
        self.theta = msg.data
        self.publish_odometry()

    def publish_odometry(self):
        current_time = self.get_clock().now().to_msg()

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Twist
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_publisher.publish(odom_msg)

        # Broadcast the transform
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info(f'Published odometry: x={self.x}, y={self.y}, theta={self.theta}, v={self.v}')
        self.get_logger().info('Broadcasted transform from odom to base_footprint.')

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
