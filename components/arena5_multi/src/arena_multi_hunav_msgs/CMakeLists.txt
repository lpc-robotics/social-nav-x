cmake_minimum_required(VERSION 3.8)
project(arena_multi_hunav_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviorModifiers.msg" "msg/RobotInfluence.msg" "msg/InteractionForces.msg"
  "srv/ComputeMultiAgents.srv"
  DEPENDENCIES hunav_msgs geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
