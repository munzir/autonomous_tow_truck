import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import csv

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_markers = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        self.waypoints = self.load_waypoints('/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/waypoints.csv')
        self.index = 0
        self.marker_array = MarkerArray()

    def load_waypoints(self, filename):
        waypoints = []
        try:
            with open(filename, mode='r') as file:
                reader = csv.reader(file)
                for row in reader:
                    # Assuming CSV format: x, y, z, w (Quaternion)
                    x, y, z, w = map(float, row)
                    waypoints.append((x, y, z, w))
        except Exception as e:
            self.get_logger().error(f"Error reading waypoints: {e}")
        return waypoints

    def publish_waypoints(self):
        if self.index < len(self.waypoints):
            waypoint = self.waypoints[self.index]
            marker = Marker()

            # Fill header with timestamp and frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"  # You can adjust this as needed

            # Marker ID and namespace
            marker.ns = "waypoint_markers"
            marker.id = self.index

            # Set the pose (position and orientation)
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.0  # You can modify this if needed
            marker.pose.orientation.x = waypoint[2]
            marker.pose.orientation.y = 0.0  # Assuming y is 0 for simplicity
            marker.pose.orientation.z = 0.0  # Assuming z is 0 for simplicity
            marker.pose.orientation.w = waypoint[3]

            # Set marker type (e.g., SPHERE)
            marker.type = Marker.SPHERE

            # Set marker scale
            marker.scale.x = 1.0  # Size of the marker
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            # Set color (green for waypoints)
            marker.color.r = 0.0
            marker.color.g = 255.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Full opacity

            # Add marker to MarkerArray
            self.marker_array.markers.append(marker)

            # Publish the MarkerArray containing all markers so far
            self.publisher_markers.publish(self.marker_array)

            self.get_logger().info(f"Published waypoint {self.index + 1}: {waypoint}")
            self.index += 1
        else:
            self.get_logger().info("All waypoints published.")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

