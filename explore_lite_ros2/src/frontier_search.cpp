#include "explore_lite_ros2/frontier_search.hpp"
#include "explore_lite_ros2/costmap_tools.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace explore_lite
{

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale,
                               double gain_scale,
                               double min_frontier_size,
                               double max_frontier_distance)
  : costmap_(costmap),
    potential_scale_(potential_scale),
    gain_scale_(gain_scale),
    min_frontier_size_(min_frontier_size),
    max_frontier_distance_(max_frontier_distance)
{}

std::vector<Frontier> FrontierSearch::searchFrom(const geometry_msgs::msg::Point& position)
{
  std::vector<Frontier> frontier_list;

  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my))
    return frontier_list;

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);
  std::queue<unsigned int> bfs;

  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, nav2_costmap_2d::FREE_SPACE, *costmap_))
    bfs.push(clear);
  else
    bfs.push(pos);
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    for (unsigned nbr : nhood4(idx, *costmap_)) {
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_ &&
            new_frontier.min_distance <= max_frontier_distance_)
        {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  for (auto& f : frontier_list) f.cost = frontierCost(f);
  std::sort(frontier_list.begin(), frontier_list.end(),
            [](auto& a, auto& b) { return a.cost < b.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  Frontier output;
  output.centroid.x = output.centroid.y = 0;
  output.size = 1;
  output.min_distance = 1e9;

  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  unsigned int rx, ry;
  double rx_w, ry_w;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, rx_w, ry_w);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point p;
        p.x = wx;
        p.y = wy;
        p.z = 0.0;
        output.points.push_back(p);

        output.size++;
        output.centroid.x += wx;
        output.centroid.y += wy;

        double d = hypot(rx_w - wx, ry_w - wy);
        if (d < output.min_distance) {
          output.min_distance = d;
          output.middle.x = wx;
          output.middle.y = wy;
        }
        bfs.push(nbr);
      }
    }
  }

  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag)
{
  if (map_[idx] != nav2_costmap_2d::NO_INFORMATION || frontier_flag[idx])
    return false;
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == nav2_costmap_2d::FREE_SPACE)
      return true;
  }
  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return potential_scale_ * frontier.min_distance - gain_scale_ * frontier.size * costmap_->getResolution();
}

bool FrontierSearch::isFrontierReachable(const geometry_msgs::msg::Point& robot, const Frontier& f)
{
  return true;
}

bool FrontierSearch::isCellReachable(unsigned int start, unsigned int target)
{
  return true;
}

}
