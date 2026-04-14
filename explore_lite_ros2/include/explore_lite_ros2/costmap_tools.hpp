#pragma once

#include <queue>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>

namespace explore_lite
{
inline std::vector<unsigned int> nhood4(unsigned int idx,
                                        const nav2_costmap_2d::Costmap2D& costmap)
{
  std::vector<unsigned int> out;

  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();

  if (idx > size_x * size_y - 1) {
    RCLCPP_WARN(rclcpp::get_logger("explore_lite"), "Evaluating nhood for offmap point");
    return out;
  }

  if (idx % size_x > 0) {
    out.push_back(idx - 1);
  }
  if (idx % size_x < size_x - 1) {
    out.push_back(idx + 1);
  }
  if (idx >= size_x) {
    out.push_back(idx - size_x);
  }
  if (idx < size_x * (size_y - 1)) {
    out.push_back(idx + size_x);
  }

  return out;
}

inline std::vector<unsigned int> nhood8(unsigned int idx,
                                        const nav2_costmap_2d::Costmap2D& costmap)
{
  std::vector<unsigned int> out = nhood4(idx, costmap);

  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();

  if (idx > size_x * size_y - 1) {
    return out;
  }

  if (idx % size_x > 0 && idx >= size_x) {
    out.push_back(idx - 1 - size_x);
  }
  if (idx % size_x > 0 && idx < size_x * (size_y - 1)) {
    out.push_back(idx - 1 + size_x);
  }
  if (idx % size_x < size_x - 1 && idx >= size_x) {
    out.push_back(idx + 1 - size_x);
  }
  if (idx % size_x < size_x - 1 && idx < size_x * (size_y - 1)) {
    out.push_back(idx + 1 + size_x);
  }

  return out;
}

inline bool nearestCell(unsigned int& result,
                        unsigned int start,
                        unsigned char val,
                        const nav2_costmap_2d::Costmap2D& costmap)
{
  const unsigned char* map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return false;
  }

  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  bfs.push(start);
  visited_flag[start] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    if (map[idx] == val) {
      result = idx;
      return true;
    }

    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}
}  // namespace explore_lite