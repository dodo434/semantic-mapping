#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/header.hpp>

namespace semantic_mapping
{
class SemanticLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("Costmap lifecycle node is unavailable");
    }
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("topic", rclcpp::ParameterValue(std::string("/semantic/semantic_grid")));
    declareParameter("revision_topic", rclcpp::ParameterValue(std::string("/semantic/map_revision")));
    declareParameter("timeout", rclcpp::ParameterValue(0.75));
    node->get_parameter(getFullName("enabled"), enabled_);
    const auto topic = node->get_parameter(getFullName("topic")).as_string();
    const auto revision_topic = node->get_parameter(getFullName("revision_topic")).as_string();
    timeout_ = node->get_parameter(getFullName("timeout")).as_double();
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
      throw std::runtime_error("Semantic costmap timeout must be positive");
    }
    world_ = layered_costmap_->getGlobalFrameID();
    clock_ = node->get_clock();
    current_ = false;
    subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid) {
        const auto & origin = grid->info.origin;
        if (grid->header.frame_id != world_ || !std::isfinite(grid->info.resolution) ||
          grid->info.resolution <= 0.0 || !std::isfinite(origin.position.x) ||
          !std::isfinite(origin.position.y) || std::abs(origin.orientation.w - 1) > 1e-6 ||
          origin.orientation.x != 0 || origin.orientation.y != 0 || origin.orientation.z != 0 ||
          grid->info.width == 0 || grid->info.height == 0 ||
          static_cast<size_t>(grid->info.width) * grid->info.height != grid->data.size())
        {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (to_ns(grid->header.stamp) >= revision_ns_) {
          latest_ = std::move(grid);
        }
      });
    revision_ = node->create_subscription<std_msgs::msg::Header>(
      revision_topic, rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Header::ConstSharedPtr header) {
        std::lock_guard<std::mutex> lock(mutex_);
        revision_ns_ = to_ns(header->stamp);
        if (latest_ && to_ns(latest_->header.stamp) < revision_ns_) {
          latest_.reset();
        }
      });
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
  }

  bool isClearable() override {return false;}

  void updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_) {
      current_ = true;
      return;
    }
    auto * master = layered_costmap_->getCostmap();
    const auto resolution = master->getResolution();
    *min_x = std::min(*min_x, master->getOriginX());
    *min_y = std::min(*min_y, master->getOriginY());
    *max_x = std::max(*max_x, master->getOriginX() + resolution * master->getSizeInCellsX());
    *max_y = std::max(*max_y, master->getOriginY() + resolution * master->getSizeInCellsY());
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master, int x0, int y0, int x1, int y1) override
  {
    if (!enabled_) {
      return;
    }
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      grid = latest_;
    }
    const auto age = grid ? (clock_->now().nanoseconds() - to_ns(grid->header.stamp)) * 1e-9 : 1e9;
    current_ = grid && age >= -0.05 && age <= timeout_;
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        double wx, wy;
        master.mapToWorld(x, y, wx, wy);
        const int8_t evidence = current_ ? lookup(*grid, wx, wy) : -1;
        if (evidence != 0) {
          master.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        } else if (master.getCost(x, y) == nav2_costmap_2d::NO_INFORMATION) {
          master.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
        }
      }
    }
  }

private:
  static int64_t to_ns(const builtin_interfaces::msg::Time & time)
  {
    return static_cast<int64_t>(time.sec) * 1000000000 + time.nanosec;
  }

  static int8_t lookup(const nav_msgs::msg::OccupancyGrid & grid, double x, double y)
  {
    const auto ix = std::floor((x - grid.info.origin.position.x) / grid.info.resolution);
    const auto iy = std::floor((y - grid.info.origin.position.y) / grid.info.resolution);
    if (ix < 0 || iy < 0 || ix >= grid.info.width || iy >= grid.info.height) {
      return -1;
    }
    return grid.data[static_cast<size_t>(iy) * grid.info.width + static_cast<size_t>(ix)];
  }

  std::mutex mutex_;
  std::string world_;
  int64_t revision_ns_ = 0;
  double timeout_ = 0.75;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr revision_;
};
}  // namespace semantic_mapping

PLUGINLIB_EXPORT_CLASS(semantic_mapping::SemanticLayer, nav2_costmap_2d::Layer)
