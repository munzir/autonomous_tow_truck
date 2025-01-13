#include "behaviortree_cpp_v3/behavior_tree.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

class LoadPrecomputedPath : public BT::SyncActionNode
{
public:
    LoadPrecomputedPath(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<nav_msgs::msg::Path>("path"),
        };
    }

    BT::NodeStatus tick() override
    {
        // Load the precomputed path from the YAML file
        std::string yaml_file_path = "/root/autonomous_tow_truck/src/waypoint_publisher/waypoint_publisher/saved_path.yaml";

        // Parse the YAML file
        YAML::Node yaml = YAML::LoadFile(yaml_file_path);
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = yaml["header"]["frame_id"].as<std::string>();

        // Loop through the poses in the YAML file and create a path
        for (const auto& pose_data : yaml["poses"])
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = pose_data["header"]["frame_id"].as<std::string>();

            pose_stamped.pose.position.x = pose_data["position"]["x"].as<double>();
            pose_stamped.pose.position.y = pose_data["position"]["y"].as<double>();
            pose_stamped.pose.position.z = pose_data["position"]["z"].as<double>();

            pose_stamped.pose.orientation.w = pose_data["orientation"]["w"].as<double>();
            pose_stamped.pose.orientation.x = pose_data["orientation"]["x"].as<double>();
            pose_stamped.pose.orientation.y = pose_data["orientation"]["y"].as<double>();
            pose_stamped.pose.orientation.z = pose_data["orientation"]["z"].as<double>();

            path_msg.poses.push_back(pose_stamped);
        }

        // Set the path as an output
        setOutput("path", path_msg);

        return BT::NodeStatus::SUCCESS;
    }
};

// Register the custom node
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LoadPrecomputedPath>("LoadPrecomputedPath");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create your ROS 2 node
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("load_precomputed_path_node");

    // Add a callback group for handling the behavior tree execution or additional tasks
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // Start spinning the node to keep processing events
    exec.spin();

    rclcpp::shutdown();
    return 0;
}

