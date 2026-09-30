#include "arena_peer_costmap/peer_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arena_peer_costmap
{

bool pointInOrientedBox(
  double point_x, double point_y, const PeerPose & pose,
  double half_length, double half_width)
{
  const double dx = point_x - pose.x;
  const double dy = point_y - pose.y;
  const double local_x = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
  const double local_y = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
  return std::abs(local_x) <= half_length && std::abs(local_y) <= half_width;
}

void PeerObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PeerObstacleLayer could not lock lifecycle node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("peers", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("half_length", rclcpp::ParameterValue(0.24));
  declareParameter("half_width", rclcpp::ParameterValue(0.22));
  declareParameter("timeout", rclcpp::ParameterValue(0.6));
  std::vector<std::string> peers;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".peers", peers);
  node->get_parameter(name_ + ".half_length", half_length_);
  node->get_parameter(name_ + ".half_width", half_width_);
  node->get_parameter(name_ + ".timeout", timeout_);
  rolling_window_ = layered_costmap_->isRolling();
  if (half_length_ <= 0.0 || half_width_ <= 0.0 || timeout_ <= 0.0) {
    throw std::runtime_error("PeerObstacleLayer dimensions and timeout must be positive");
  }

  default_value_ = nav2_costmap_2d::FREE_SPACE;
  matchSize();
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = callback_group_;
  for (const auto & peer : peers) {
    if (peer.empty()) {
      throw std::runtime_error("PeerObstacleLayer peer names must be non-empty");
    }
    poses_.emplace(peer, PeerPose{});
    subscriptions_.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
      "/" + peer + "/odom", rclcpp::SensorDataQoS(),
      [this, peer](nav_msgs::msg::Odometry::SharedPtr message) {
        odometryCallback(peer, message);
      },
      options));
  }
  current_ = peers.empty();
  RCLCPP_INFO(logger_, "%s: tracking %zu ideal peer poses", name_.c_str(), peers.size());
}

void PeerObstacleLayer::matchSize()
{
  CostmapLayer::matchSize();
}

void PeerObstacleLayer::reset()
{
  resetMap(0, 0, size_x_, size_y_);
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & item : poses_) {
    item.second.valid = false;
  }
  current_ = poses_.empty();
}

void PeerObstacleLayer::odometryCallback(
  const std::string & peer, nav_msgs::msg::Odometry::SharedPtr message)
{
  const auto & q = message->pose.pose.orientation;
  PeerPose pose;
  pose.x = message->pose.pose.position.x;
  pose.y = message->pose.pose.position.y;
  pose.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  pose.received = std::chrono::steady_clock::now();
  pose.valid = std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
  std::lock_guard<std::mutex> lock(mutex_);
  poses_[peer] = pose;
}

std::vector<PeerPose> PeerObstacleLayer::snapshot(bool * all_fresh) const
{
  const auto now = std::chrono::steady_clock::now();
  std::vector<PeerPose> result;
  *all_fresh = true;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & item : poses_) {
    const auto & pose = item.second;
    if (!pose.valid) {
      *all_fresh = false;
      continue;
    }
    const double age = std::chrono::duration<double>(now - pose.received).count();
    if (age > timeout_) {
      *all_fresh = false;
    }
    // Keep the last known obstacle while stale. The command guard stops the robot.
    result.push_back(pose);
  }
  return result;
}

void PeerObstacleLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> map_lock(*getMutex());
  if (rolling_window_) {
    updateOrigin(
      robot_x - getSizeInMetersX() / 2.0,
      robot_y - getSizeInMetersY() / 2.0);
  }
  if (!enabled_) {
    return;
  }
  bool all_fresh = false;
  const auto poses = snapshot(&all_fresh);
  current_ = all_fresh;
  const double radius = std::hypot(half_length_, half_width_);
  double current_min_x = std::numeric_limits<double>::infinity();
  double current_min_y = std::numeric_limits<double>::infinity();
  double current_max_x = -std::numeric_limits<double>::infinity();
  double current_max_y = -std::numeric_limits<double>::infinity();
  for (const auto & pose : poses) {
    current_min_x = std::min(current_min_x, pose.x - radius);
    current_min_y = std::min(current_min_y, pose.y - radius);
    current_max_x = std::max(current_max_x, pose.x + radius);
    current_max_y = std::max(current_max_y, pose.y + radius);
  }
  if (std::isfinite(current_min_x)) {
    *min_x = std::min(*min_x, current_min_x);
    *min_y = std::min(*min_y, current_min_y);
    *max_x = std::max(*max_x, current_max_x);
    *max_y = std::max(*max_y, current_max_y);
  }
  if (std::isfinite(previous_min_x_)) {
    *min_x = std::min(*min_x, previous_min_x_);
    *min_y = std::min(*min_y, previous_min_y_);
    *max_x = std::max(*max_x, previous_max_x_);
    *max_y = std::max(*max_y, previous_max_y_);
  }
  previous_min_x_ = current_min_x;
  previous_min_y_ = current_min_y;
  previous_max_x_ = current_max_x;
  previous_max_y_ = current_max_y;
}

void PeerObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  resetMap(0, 0, size_x_, size_y_);
  bool all_fresh = false;
  const auto poses = snapshot(&all_fresh);
  current_ = all_fresh;
  for (int mx = std::max(0, min_i); mx < std::min(static_cast<int>(size_x_), max_i); ++mx) {
    for (int my = std::max(0, min_j); my < std::min(static_cast<int>(size_y_), max_j); ++my) {
      double wx = 0.0;
      double wy = 0.0;
      mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      if (std::any_of(
          poses.begin(), poses.end(),
          [&](const PeerPose & pose) {
            return pointInOrientedBox(wx, wy, pose, half_length_, half_width_);
          }))
      {
        setCost(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
          nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace arena_peer_costmap

PLUGINLIB_EXPORT_CLASS(arena_peer_costmap::PeerObstacleLayer, nav2_costmap_2d::Layer)
