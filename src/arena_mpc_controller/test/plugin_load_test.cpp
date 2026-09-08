#include <iostream>
#include <memory>

#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>

int main()
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  const auto plugin = loader.createSharedInstance("arena_mpc_controller::MpcController");
  if (!plugin) {
    std::cerr << "pluginlib returned a null controller" << std::endl;
    return 1;
  }
  std::cout << "PLUGINLIB_LOAD_OK" << std::endl;
  return 0;
}
