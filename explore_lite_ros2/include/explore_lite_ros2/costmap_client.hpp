#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>

namespace explore_lite
{
struct CostmapClientOptions
{
  std::string costmap_topic;
  std::string costmap_updates_topic;
  std::string robot_base_frame;
  double transform_tolerance;
  bool costmap_is_nav2{true};
};

class Costmap2DClient
{
public:
  Costmap2DClient(rclcpp::Node& node,
                  const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                  const CostmapClientOptions& options);

  void updateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updatePartialMap(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);
  std::optional<geometry_msgs::msg::Pose> getRobotPose() const;

  nav2_costmap_2d::Costmap2D& getCostmap() { return costmap_; }
  bool hasMap() const { return has_map_; }
  const std::string& getGlobalFrame() const { return global_frame_; }

private:
  rclcpp::Node& node_;  // 新增：节点引用用于日志
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav2_costmap_2d::Costmap2D costmap_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;
  
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  bool costmap_is_nav2_;
  bool has_map_{false};
};
}  // namespace explore_lite
