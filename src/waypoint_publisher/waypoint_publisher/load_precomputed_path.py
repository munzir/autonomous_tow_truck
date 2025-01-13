import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from behaviortree_cpp_v3 import SyncActionNode, NodeStatus

class LoadPrecomputedPath(SyncActionNode):
    def __init__(self, name, config):
        super().__init__(name, config)

    def provided_ports():
        return {
            "path": "Output: Precomputed path as a nav_msgs/Path message"
        }

    def tick(self):
        # Hardcoded file path
        yaml_file_path = "/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/saved_path.yaml"

        # Load YAML file
        with open(yaml_file_path, 'r') as file:
            yaml_content = yaml.safe_load(file)

        path_msg = Path()
        path_msg.header.frame_id = yaml_content["header"]["frame_id"]

        # Parse poses
        for pose_data in yaml_content["poses"]:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = pose_data["header"]["frame_id"]

            pose_stamped.pose.position.x = pose_data["position"]["x"]
            pose_stamped.pose.position.y = pose_data["position"]["y"]
            pose_stamped.pose.position.z = pose_data["position"]["z"]

            pose_stamped.pose.orientation.w = pose_data["orientation"]["w"]
            pose_stamped.pose.orientation.x = pose_data["orientation"]["x"]
            pose_stamped.pose.orientation.y = pose_data["orientation"]["y"]
            pose_stamped.pose.orientation.z = pose_data["orientation"]["z"]

            path_msg.poses.append(pose_stamped)

        # Set the output path
        self.set_output("path", path_msg)
        return NodeStatus.SUCCESS

# Register the node for use in behavior trees
def register_nodes(factory):
    factory.register_node_type(LoadPrecomputedPath, "LoadPrecomputedPath")

