// yolo_costmap_layer.cpp
#include "yolo_costmap_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::YoloLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

YoloLayer::YoloLayer() : current_bboxes_(), bbox_mutex_() {}

void YoloLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  
  // Parameters
  nh.param("inflation_radius", inflation_radius_, 0.5);
  
  // Class costs (example: person=254, chair=128, etc.)
  nh.param("class_costs/person", class_costs_["person"], static_cast<uint8_t>(254));
  nh.param("class_costs/chair", class_costs_["chair"], static_cast<uint8_t>(128));
  
  // Subscriber
  bbox_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 1, &YoloLayer::bboxCallback, this);
  
  matchSize();
}

void YoloLayer::bboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(bbox_mutex_);
  current_bboxes_ = msg->bounding_boxes;
}

void YoloLayer::imageToWorld(double img_x, double img_y, double depth, double& world_x, double& world_y)
{
  // Convert from normalized coordinates if needed
  if (normalized_coordinates_) {
    img_x *= image_width_;
    img_y *= image_height_;
  }
  
  // Camera projection math (similar to point cloud conversion)
  double fx = camera_info_.K[0];
  double fy = camera_info_.K[4];
  double cx = camera_info_.K[2];
  double cy = camera_info_.K[5];
  
  // Convert to 3D camera coordinates
  double z = depth;
  double x = (img_x - cx) * z / fx;
  double y = (img_y - cy) * z / fy;
  
  // Transform to world coordinates using TF
  geometry_msgs::PointStamped camera_point;
  camera_point.header.frame_id = "camera_depth_optical_frame";
  camera_point.point.x = x;
  camera_point.point.y = y;
  camera_point.point.z = z;
  
  geometry_msgs::PointStamped world_point;
  try {
    tf_.transformPoint(global_frame_, camera_point, world_point);
    world_x = world_point.point.x;
    world_y = world_point.point.y;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Failed to transform point: %s", ex.what());
  }
}

void YoloLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(bbox_mutex_);
  
  if (current_bboxes_.empty()) {
    return;
  }
  
  // Expand bounds to include all detected objects
  for (const auto& bbox : current_bboxes_) {
    // Convert from image coordinates to world coordinates
    // (You'll need camera projection logic here)
    double wx, wy;
    imageToWorld(bbox.x, bbox.y, bbox.depth, wx, wy);
    
    // Expand bounds with inflation radius
    *min_x = std::min(*min_x, wx - inflation_radius_);
    *min_y = std::min(*min_y, wy - inflation_radius_);
    *max_x = std::max(*max_x, wx + inflation_radius_);
    *max_y = std::max(*max_y, wy + inflation_radius_);
  }
}

void YoloLayer::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(bbox_mutex_);
  
  if (!enabled_ || current_bboxes_.empty()) {
    return;
  }
  
  // Process each bounding box
  for (const auto& bbox : current_bboxes_) {
    // Get cost for this class (default to LETHAL_OBSTACLE if not specified)
    uint8_t cost = class_costs_.count(bbox.Class) ? class_costs_[bbox.Class] : nav2_costmap_2d::LETHAL_OBSTACLE;
    
    // Convert to world coordinates
    double wx, wy;
    imageToWorld(bbox.x, bbox.y, bbox.depth, wx, wy);
    
    // Mark the area in costmap
    unsigned int mx, my;
    if (worldToMap(wx, wy, mx, my)) {
      // Apply circular inflation around the detection
      for (int i = -inflation_radius_; i <= inflation_radius_; ++i) {
        for (int j = -inflation_radius_; j <= inflation_radius_; ++j) {
          if (i*i + j*j <= inflation_radius_*inflation_radius_) {
            unsigned int index = getIndex(mx + i, my + j);
            master_grid.setCost(mx + i, my + j, cost);
          }
        }
      }
    }
  }
}

}  // namespace nav2_costmap_2d



