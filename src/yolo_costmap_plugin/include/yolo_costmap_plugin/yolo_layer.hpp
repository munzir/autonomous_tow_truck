#ifndef YOLO_COSTMAP_PLUGIN__YOLO_LAYER_HPP_
#define YOLO_COSTMAP_PLUGIN__YOLO_LAYER_HPP_

#include <mutex>
#include <string>
#include <map>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "rclcpp/rclcpp.hpp"

namespace yolo_costmap_plugin
{

class YoloLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  YoloLayer();
  virtual ~YoloLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);
    
private:
  void bboxCallback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg);
  
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bbox_sub_;
  std::vector<darknet_ros_msgs::msg::BoundingBox> current_bboxes_;
  std::mutex bbox_mutex_;
  double inflation_radius_;
  std::map<std::string, uint8_t> class_costs_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace yolo_costmap_plugin

#endif  // YOLO_COSTMAP_PLUGIN__YOLO_LAYER_HPP_
