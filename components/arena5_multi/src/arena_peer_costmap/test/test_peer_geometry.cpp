#include <cmath>
#include <iostream>

#include "arena_peer_costmap/peer_obstacle_layer.hpp"

using arena_peer_costmap::PeerPose;
using arena_peer_costmap::pointInOrientedBox;

int main()
{
  PeerPose pose;
  pose.x = 2.0;
  pose.y = 3.0;
  if (!pointInOrientedBox(2.23, 3.21, pose, 0.24, 0.22) ||
    pointInOrientedBox(2.241, 3.0, pose, 0.24, 0.22))
  {
    std::cerr << "axis-aligned footprint check failed\n";
    return 1;
  }
  pose.x = 1.0;
  pose.y = -2.0;
  pose.yaw = std::acos(-1.0) / 2.0;
  if (!pointInOrientedBox(1.0, -1.77, pose, 0.24, 0.22) ||
    pointInOrientedBox(1.0, -1.759, pose, 0.24, 0.22) ||
    !pointInOrientedBox(0.79, -2.0, pose, 0.24, 0.22))
  {
    std::cerr << "rotated footprint check failed\n";
    return 1;
  }
  return 0;
}
