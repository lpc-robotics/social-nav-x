cmake_minimum_required(VERSION 3.8)
project(arena_peer_costmap)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(arena_peer_costmap SHARED src/peer_obstacle_layer.cpp)
target_compile_features(arena_peer_costmap PUBLIC cxx_std_17)
target_include_directories(arena_peer_costmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arena_peer_costmap nav2_costmap_2d nav_msgs pluginlib rclcpp)

pluginlib_export_plugin_description_file(nav2_costmap_2d peer_costmap_plugins.xml)
install(TARGETS arena_peer_costmap EXPORT export_arena_peer_costmap
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES peer_costmap_plugins.xml DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_arena_peer_costmap HAS_LIBRARY_TARGET)
ament_export_dependencies(nav2_costmap_2d nav_msgs pluginlib rclcpp)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_peer_geometry test/test_peer_geometry.cpp)
  target_link_libraries(test_peer_geometry arena_peer_costmap)
  add_test(NAME peer_geometry COMMAND test_peer_geometry)
endif()

ament_package()
