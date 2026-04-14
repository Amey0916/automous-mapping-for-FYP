#include "explore_lite_ros2/explore_node.hpp"
#include "explore_lite_ros2/costmap_client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>

namespace explore_lite
{
using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

ExploreNode::ExploreNode() : Node("explore_lite_node"), prev_distance_{0.0}
{
  prev_goal_.x = 0.0;
  prev_goal_.y = 0.0;
  prev_goal_.z = 0.0;

  this->declare_parameter("planner_frequency", 0.2);
  this->declare_parameter("progress_timeout", 60.0);
  this->declare_parameter("visualize", true);
  this->declare_parameter("potential_scale", 1e-3);
  this->declare_parameter("gain_scale", 1.0);
  this->declare_parameter("min_frontier_size", 0.5);
  this->declare_parameter("max_frontier_distance", 10.0);
  this->declare_parameter("costmap_topic", "global_costmap/costmap");
  this->declare_parameter("costmap_updates_topic", "global_costmap/costmap_updates");
  this->declare_parameter("robot_base_frame", "base_link");
  this->declare_parameter("transform_tolerance", 0.3);
  this->declare_parameter("action_server_name", "navigate_to_pose");
  this->declare_parameter("blacklist_timeout", 10.0);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", progress_timeout_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size_);
  this->get_parameter("max_frontier_distance", max_frontier_distance_);
  this->get_parameter("action_server_name", action_server_name_);
  this->get_parameter("blacklist_timeout", blacklist_timeout_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  CostmapClientOptions options;
  options.costmap_topic = this->get_parameter("costmap_topic").as_string();
  options.costmap_updates_topic = this->get_parameter("costmap_updates_topic").as_string();
  options.robot_base_frame = this->get_parameter("robot_base_frame").as_string();
  options.transform_tolerance = this->get_parameter("transform_tolerance").as_double();
  costmap_client_ = std::make_unique<Costmap2DClient>(*this, tf_buffer_, options);

  search_ = FrontierSearch(
      &costmap_client_->getCostmap(),
      potential_scale_,
      gain_scale_,
      min_frontier_size_,
      max_frontier_distance_
  );

  nav_client_ = rclcpp_action::create_client<NavigateToPoseAction>(this, action_server_name_);

  RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
  nav_client_->wait_for_action_server(std::chrono::seconds(10));
  RCLCPP_INFO(this->get_logger(), "Nav2 connected!");

  if (visualize_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/frontiers", rclcpp::QoS(1));
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / planner_frequency_),
      std::bind(&ExploreNode::makePlan, this));

  last_progress_ = this->now();
}

void ExploreNode::makePlan()
{
  if (exploring_timer_->is_canceled()) return;

  auto robot_pose_opt = costmap_client_->getRobotPose();
  if (!robot_pose_opt) {
    RCLCPP_WARN(this->get_logger(), "Cannot get robot pose");
    return;
  }
  geometry_msgs::msg::Pose robot_pose = *robot_pose_opt;

  std::vector<Frontier> frontiers = search_.searchFrom(robot_pose.position);
  if (frontiers.empty()) {
    RCLCPP_INFO(this->get_logger(), "Exploration done!");
    return;
  }

  std::vector<Frontier> valid_frontiers;
  for (auto& f : frontiers) {
    if (!goalOnBlacklist(f.centroid)) valid_frontiers.push_back(f);
  }

  if (valid_frontiers.empty()) {
    RCLCPP_WARN(this->get_logger(), "All frontiers blacklisted, will retry...");
    return;
  }
  // 按距离排序，优先选最近的目标（避免选到太远/障碍里的点）
  std::sort(valid_frontiers.begin(), valid_frontiers.end(),
            [&robot_pose](const Frontier& a, const Frontier& b) {
              double d1 = hypot(a.centroid.x - robot_pose.position.x, a.centroid.y - robot_pose.position.y);
              double d2 = hypot(b.centroid.x - robot_pose.position.x, b.centroid.y - robot_pose.position.y);
              return d1 < d2;
            });
  Frontier best = valid_frontiers[0];

  NavigateToPoseAction::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position = best.centroid;
  goal_msg.pose.pose.orientation = robot_pose.orientation;

  auto opt = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
  opt.result_callback = std::bind(&ExploreNode::reachedGoal, this, std::placeholders::_1);
  nav_client_->async_send_goal(goal_msg, opt);

  if (visualize_) visualizeFrontiers(frontiers);
  RCLCPP_INFO(this->get_logger(), "Send goal: %.2f %.2f", best.centroid.x, best.centroid.y);
}

void ExploreNode::reachedGoal(const ClientGoalHandle::WrappedResult& result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(this->get_logger(), "Goal failed, blacklist");
    BlacklistedGoal bg;
    bg.goal = prev_goal_;
    bg.blacklist_time = this->now();
    frontier_blacklist_.push_back(bg);
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal success!");
  }
}

bool ExploreNode::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  rclcpp::Time now = this->now();
  bool found = false;

  for (auto& bg : frontier_blacklist_) {
    if (samePoint(bg.goal, goal) && (now - bg.blacklist_time).seconds() < blacklist_timeout_) {
      found = true;
    }
  }

  frontier_blacklist_.erase(
    std::remove_if(frontier_blacklist_.begin(), frontier_blacklist_.end(),
      [this, now](const BlacklistedGoal& bg) {
        return (now - bg.blacklist_time).seconds() >= blacklist_timeout_;
      }),
    frontier_blacklist_.end()
  );

  return found;
}

bool ExploreNode::samePoint(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const
{
  const double t = 0.15;
  return fabs(a.x - b.x) < t && fabs(a.y - b.y) < t;
}

void ExploreNode::visualizeFrontiers(const std::vector<Frontier>& f)
{
  visualization_msgs::msg::MarkerArray m;
  visualization_msgs::msg::Marker del;
  del.action = 3;
  m.markers.push_back(del);
  marker_pub_->publish(m);
}

void ExploreNode::stop()
{
  exploring_timer_->cancel();
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<explore_lite::ExploreNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
