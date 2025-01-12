import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from time import sleep
import csv

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.global_path = Path()
        self.global_path.header.frame_id = 'map'
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        
        # Load waypoints from the CSV file
        self.waypoints = self.load_waypoints('/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints.csv')
        
        # Initialize the current pose (robot's initial pose)
        self.current_pose.pose.position.x = self.waypoints[0][0]
        self.current_pose.pose.position.y = self.waypoints[0][1]
        self.current_pose.pose.orientation.w = self.waypoints[0][2]
        self.current_pose.pose.orientation.z = self.waypoints[0][3]
        
        # Ensure global costmap and map are loaded
        self.get_costmap_and_map()

    def load_waypoints(self, filepath):
        """Load waypoints from a CSV file."""
        waypoints = []
        with open(filepath, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                x, y, w, z = map(float, row)
                waypoints.append((x, y, w, z))
        return waypoints

    def get_costmap_and_map(self):
        """Ensure that the costmap and map are available."""
        # This is a placeholder. In a real implementation, you can subscribe to the costmap
        # and the map topics and verify that they are loaded.
        self.get_logger().info("Ensuring global costmap and map are available...")

    def generate_path_segment(self, start_pose, goal_pose):
        """Use Nav2 to generate a path segment between start_pose and goal_pose."""
        self.get_logger().info(f"Generating path from {start_pose.pose.position.x}, {start_pose.pose.position.y} to {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")
        
        # Wait for action server to be ready
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server to be ready...')
        
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        future = self.client.send_goal_async(goal)
        
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            # Path was generated successfully, add it to the global path
            segment_path = result.result.path
            self.global_path.poses.extend(segment_path.poses)
            self.get_logger().info(f"Path segment added. {len(segment_path.poses)} waypoints added.")
            self.get_logger().debug(f"Path result: {result.result.path}")
        else:
            self.get_logger().error('Failed to generate path segment. Debug info: {}'.format(result))

    def plan_full_path(self):
        """Generate the full path by launching Nav2 multiple times."""
        for i in range(len(self.waypoints) - 1):
            start_pose = self.current_pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = self.waypoints[i + 1][0]
            goal_pose.pose.position.y = self.waypoints[i + 1][1]
            goal_pose.pose.orientation.w = self.waypoints[i + 1][2]
            goal_pose.pose.orientation.z = self.waypoints[i + 1][3]
            
            # Generate path segment
            self.generate_path_segment(start_pose, goal_pose)
            
            # Update the current pose to be the goal pose of the previous segment
            self.current_pose = goal_pose

        self.get_logger().info(f"Full path generated with {len(self.global_path.poses)} waypoints.")

    def publish_full_path(self):
        """Publish the full path to the '/full_path' topic."""
        path_publisher = self.create_publisher(Path, '/full_path', 10)
        path_publisher.publish(self.global_path)
        self.get_logger().info("Full path published.")

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlanner()

    # Plan the full path by generating path segments
    planner.plan_full_path()

    # Publish the final path
    planner.publish_full_path()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

