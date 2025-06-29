import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

# Define waypoints and thresholds
WAYPOINTS = {
    "indoor_zone": (2.0, 3.0),  # x, y coordinates in the map
    "outdoor_zone": (10.0, 5.0)
}
THRESHOLD_DISTANCE = 1.0  # Threshold for switching modes (meters)

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        # Current mode (indoor or outdoor)
        self.current_mode = 'indoor'

        # Subscribe to robot's position in the map frame
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/amcl_pose',  # Topic with the robot's pose in the map frame
            self.position_callback,
            10
        )

        # Initialize service clients for lifecycle management
        self.amcl_client = self.create_client(ChangeState, '/amcl/change_state')
        self.navsat_client = self.create_client(ChangeState, '/navsat_transform_node/change_state')

        # Wait for lifecycle services to be available
        self.wait_for_service(self.amcl_client, '/amcl/change_state')
        self.wait_for_service(self.navsat_client, '/navsat_transform_node/change_state')

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {service_name} service...')

    def position_callback(self, msg):
        # Extract robot's position
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Check proximity to waypoints and switch modes
        if self.is_near_waypoint(x, y, "outdoor_zone") and self.current_mode != 'outdoor':
            self.switch_to_outdoor()
        elif self.is_near_waypoint(x, y, "indoor_zone") and self.current_mode != 'indoor':
            self.switch_to_indoor()

    def is_near_waypoint(self, x, y, zone_name):
        waypoint = WAYPOINTS[zone_name]
        distance = ((x - waypoint[0])**2 + (y - waypoint[1])**2)**0.5
        return distance < THRESHOLD_DISTANCE

    def switch_to_indoor(self):
        self.get_logger().info('Switching to indoor mode')

        # Activate AMCL and deactivate NavSat
        self.change_state(self.amcl_client, Transition.TRANSITION_ACTIVATE)
        self.change_state(self.navsat_client, Transition.TRANSITION_DEACTIVATE)

        self.current_mode = 'indoor'

    def switch_to_outdoor(self):
        self.get_logger().info('Switching to outdoor mode')

        # Deactivate AMCL and activate NavSat
        self.change_state(self.amcl_client, Transition.TRANSITION_DEACTIVATE)
        self.change_state(self.navsat_client, Transition.TRANSITION_ACTIVATE)

        self.current_mode = 'outdoor'

    def change_state(self, client, transition_id):
        # Send a lifecycle transition request
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Successfully changed state with transition ID {transition_id}')
        else:
            self.get_logger().error(f'Failed to change state with transition ID {transition_id}')


def main(args=None):
    rclpy.init(args=args)
    supervisor_node = SupervisorNode()
    rclpy.spin(supervisor_node)
    supervisor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
