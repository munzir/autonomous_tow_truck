#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class ModelPluginTutorial : public ModelPlugin
{
public:
  ModelPluginTutorial() : ModelPlugin()
  {
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    // Create a ROS 2 node using the Gazebo ROS interface
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Ensure the node is initialized properly
    if (!ros_node_)
    {
      RCLCPP_FATAL(rclcpp::get_logger("ModelPluginTutorial"), 
                   "Failed to initialize ROS 2 node for Gazebo.");
      return;
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Hello World from ROS 2 Gazebo model plugin!");
  }

private:
  gazebo_ros::Node::SharedPtr ros_node_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPluginTutorial)
}

