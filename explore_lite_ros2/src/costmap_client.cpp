#include "explore_lite_ros2/costmap_client.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace explore_lite
{
std::array<unsigned char, 256> initTranslationTable()
{
  std::array<unsigned char, 256> cost_translation_table{};
  for (size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }
  cost_translation_table[0] = 0;
  cost_translation_table[99] = 253;
  cost_translation_table[100] = 254;
  cost_translation_table[static_cast<unsigned char>(-1)] = 255;
  return cost_translation_table;
}

static const std::array<unsigned char, 256> kCostTranslationTable =
    initTranslationTable();

Costmap2DClient::Costmap2DClient(rclcpp::Node& node,
                                 const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                 const CostmapClientOptions& options)
  : tf_buffer_(tf_buffer),
    robot_base_frame_(options.robot_base_frame),
    transform_tolerance_(options.transform_tolerance),
    node_(node)  // 新增：保存节点引用用于日志
{
  costmap_sub_ = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
      options.costmap_topic, rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        updateFullMap(msg);
      });

  costmap_updates_sub_ = node.create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      options.costmap_updates_topic, rclcpp::QoS(10),
      [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
        updatePartialMap(msg);
      });

  RCLCPP_INFO(node.get_logger(), "Costmap2DClient subscribed to %s and %s",
              options.costmap_topic.c_str(), options.costmap_updates_topic.c_str());
}

void Costmap2DClient::updateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  global_frame_ = msg->header.frame_id;

  const unsigned int size_in_cells_x = msg->info.width;
  const unsigned int size_in_cells_y = msg->info.height;
  const double resolution = msg->info.resolution;
  const double origin_x = msg->info.origin.position.x;
  const double origin_y = msg->info.origin.position.y;

  costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x, origin_y);

  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  unsigned char* costmap_data = costmap_.getCharMap();
  const size_t costmap_size =
      costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();

  for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
    unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
    costmap_data[i] = kCostTranslationTable[cell_cost];
  }

  // 新增：打印地图尺寸日志
  RCLCPP_INFO(node_.get_logger(), 
              "Costmap full update: size=(%d×%d) cells, resolution=%.3fm, origin=(%.2f,%.2f)",
              size_in_cells_x, size_in_cells_y, resolution, origin_x, origin_y);

  has_map_ = true;
}

void Costmap2DClient::updatePartialMap(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
{
  if (msg->x < 0 || msg->y < 0) {
    RCLCPP_WARN(node_.get_logger(), 
                "Costmap partial update: x=%d/y=%d is negative, skip", msg->x, msg->y);
    return;
  }

  global_frame_ = msg->header.frame_id;

  const size_t x0 = static_cast<size_t>(msg->x);
  const size_t y0 = static_cast<size_t>(msg->y);
  const size_t xn = msg->width + x0;
  const size_t yn = msg->height + y0;

  auto* mutex = costmap_.getMutex();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

  const size_t costmap_xn = costmap_.getSizeInCellsX();
  const size_t costmap_yn = costmap_.getSizeInCellsY();

  // 新增：越界日志提示
  if (xn > costmap_xn || yn > costmap_yn) {
    RCLCPP_WARN(node_.get_logger(),
                "Costmap partial update out of bounds! Update(x0=%zu,y0=%zu,xn=%zu,yn=%zu) > Costmap(size_x=%zu,size_y=%zu)",
                x0, y0, xn, yn, costmap_xn, costmap_yn);
  }

  unsigned char* costmap_data = costmap_.getCharMap();
  size_t i = 0;
  for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
    for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
      size_t idx = costmap_.getIndex(x, y);
      unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
      costmap_data[idx] = kCostTranslationTable[cell_cost];
      ++i;
    }
  }

  // 新增：提示截断的数量
  if (i < msg->data.size()) {
    RCLCPP_WARN(node_.get_logger(),
                "Costmap partial update truncated: %zu of %zu cells processed (out of bounds)",
                i, msg->data.size());
  }
}

std::optional<geometry_msgs::msg::Pose> Costmap2DClient::getRobotPose() const
{
  if (!has_map_ || global_frame_.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);

  try {
    auto transformed = tf_buffer_->transform(
        robot_pose, global_frame_, tf2::durationFromSec(transform_tolerance_));
    return transformed.pose;
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(node_.get_logger(), "Failed to transform robot pose: %s", e.what());
    return std::nullopt;
  }
}
}  // namespace explore_lite
