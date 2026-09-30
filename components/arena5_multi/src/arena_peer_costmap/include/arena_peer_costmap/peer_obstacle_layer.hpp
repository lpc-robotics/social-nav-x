#ifndef ARENA_PEER_COSTMAP__PEER_OBSTACLE_LAYER_HPP_
#define ARENA_PEER_COSTMAP__PEER_OBSTACLE_LAYER_HPP_

#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arena_peer_costmap
{

struct PeerPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::chrono::steady_clock::time_point received{};
  bool valid{false};
};

bool pointInOrientedBox(
  double point_x, double point_y, const PeerPose & pose,
  double half_length, double half_width);

class PeerObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  PeerObstacleLayer() = default;
  void onInitialize() override;
  void matchSize() override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  void odometryCallback(const std::string & peer, nav_msgs::msg::Odometry::SharedPtr message);
  std::vector<PeerPose> snapshot(bool * all_fresh) const;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, PeerPose> poses_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriptions_;
  double half_length_{0.24};
  double half_width_{0.22};
  double timeout_{0.6};
  bool rolling_window_{false};
  double previous_min_x_{std::numeric_limits<double>::infinity()};
  double previous_min_y_{std::numeric_limits<double>::infinity()};
  double previous_max_x_{-std::numeric_limits<double>::infinity()};
  double previous_max_y_{-std::numeric_limits<double>::infinity()};
};

}  // namespace arena_peer_costmap

#endif  // ARENA_PEER_COSTMAP__PEER_OBSTACLE_LAYER_HPP_
