#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    // Create a ROS 2 node using the Gazebo ROS interface
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Ensure the node is initialized properly
    if (!ros_node_)
    {
      RCLCPP_FATAL(rclcpp::get_logger("WorldPluginTutorial"), 
                   "Failed to initialize ROS 2 node for Gazebo.");
      return;
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Hello World from ROS 2 Gazebo plugin!");
  }

private:
  gazebo_ros::Node::SharedPtr ros_node_;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}

