#pragma once
#include "arena_multi_hunav_msgs/srv/compute_multi_agents.hpp"
namespace arena_multi_hunav_core {
struct Config {
  double width = 30.0, height = 23.0;
  double robot_clearance = 0.1, near_gain = 10.0, near_sigma = 0.2;
  double max_acceleration = 3.0, max_speed = 1.0;
};
using Compute = arena_multi_hunav_msgs::srv::ComputeMultiAgents;
// Pure transaction: no shared state, no motion of externally controlled robots.
Compute::Response integrate(const Compute::Request & request, const Config & config);
}
