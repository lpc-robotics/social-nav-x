#include "arena_multi_hunav_core/core.hpp"
#include <rclcpp/rclcpp.hpp>
using namespace arena_multi_hunav_core;
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("multi_sfm_server");
  Config cfg;
  cfg.width = node->declare_parameter("world_width", 30.0);
  cfg.height = node->declare_parameter("world_height", 23.0);
  cfg.robot_clearance = node->declare_parameter("robot_clearance", 0.1);
  cfg.near_gain = node->declare_parameter("near_gain", 10.0);
  cfg.near_sigma = node->declare_parameter("near_sigma", 0.2);
  cfg.max_acceleration = node->declare_parameter("max_acceleration", 3.0);
  cfg.max_speed = node->declare_parameter("max_speed", 1.0);
  auto service = node->create_service<Compute>("/multirobot/hunav/compute_agents",
    [cfg](const std::shared_ptr<Compute::Request> req, std::shared_ptr<Compute::Response> res) {
      *res = integrate(*req, cfg);
    });
  rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
