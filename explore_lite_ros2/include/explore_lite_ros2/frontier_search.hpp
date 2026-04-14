#pragma once

#include <vector>
#include <cstdint>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace explore_lite
{
struct Frontier
{
  std::uint32_t size;
  double min_distance;
  double cost;
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  std::vector<geometry_msgs::msg::Point> points;
};

class FrontierSearch
{
public:
  FrontierSearch() = default;

  FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                 double potential_scale,
                 double gain_scale,
                 double min_frontier_size,
                 double max_frontier_distance);

  std::vector<Frontier> searchFrom(const geometry_msgs::msg::Point& position);

  bool isFrontierReachable(const geometry_msgs::msg::Point& robot_pose, 
                           const Frontier& frontier);

private:
  Frontier buildNewFrontier(unsigned int initial_cell,
                            unsigned int reference,
                            std::vector<bool>& frontier_flag);

  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  double frontierCost(const Frontier& frontier);
  bool isCellReachable(unsigned int start_idx, unsigned int target_idx);

  nav2_costmap_2d::Costmap2D* costmap_{nullptr};
  unsigned char* map_{nullptr};
  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double potential_scale_{1e-3};
  double gain_scale_{1.0};
  double min_frontier_size_{0.5};
  double max_frontier_distance_{5.0};
};
}
