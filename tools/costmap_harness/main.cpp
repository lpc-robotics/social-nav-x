#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// The stock Humble executable hardcodes /costmap. This test-only launcher
// permits separate namespaces for two instances of the installed costmap.
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
