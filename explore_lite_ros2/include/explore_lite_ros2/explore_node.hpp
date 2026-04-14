#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <vector>

#include "explore_lite_ros2/costmap_client.hpp"
#include "explore_lite_ros2/frontier_search.hpp"

namespace explore_lite
{

struct BlacklistedGoal
{
  geometry_msgs::msg::Point goal;
  rclcpp::Time blacklist_time;
};

class ExploreNode : public rclcpp::Node
{
public:
  ExploreNode();
  void stop();

private:
  using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

  void makePlan();
  void visualizeFrontiers(const std::vector<Frontier>& frontiers);
  void reachedGoal(const ClientGoalHandle::WrappedResult& result);
  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  bool samePoint(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;

  // 参数
  double planner_frequency_{0.2};
  double progress_timeout_{60.0};
  bool visualize_{true};
  double potential_scale_{1e-3};
  double gain_scale_{1.0};
  double min_frontier_size_{0.5};
  double max_frontier_distance_{5.0};
  std::string action_server_name_{"navigate_to_pose"};
  double blacklist_timeout_{10.0}; // 10秒自动过期

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 代价地图
  std::unique_ptr<Costmap2DClient> costmap_client_;

  // 前沿搜索
  FrontierSearch search_;

  // 导航客户端
  rclcpp_action::Client<NavigateToPoseAction>::SharedPtr nav_client_;

  // 可视化
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr exploring_timer_;

  // 状态
  rclcpp::Time last_progress_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_{0.0};

  std::vector<BlacklistedGoal> frontier_blacklist_;
};

}
