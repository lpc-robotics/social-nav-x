cmake_minimum_required(VERSION 3.16)
project(arena_mpc_controller VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(arena_mpc_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/mpc_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  arena_mpc_core
  geometry_msgs
  hunav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(mpc_command_watchdog src/mpc_command_watchdog.cpp)
target_compile_features(mpc_command_watchdog PRIVATE cxx_std_17)
target_compile_options(mpc_command_watchdog PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(mpc_command_watchdog
  geometry_msgs hunav_msgs nav2_msgs nav_msgs rclcpp sensor_msgs std_msgs
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  BUILD_RPATH_USE_ORIGIN ON
  INSTALL_RPATH "$ORIGIN"
)
pluginlib_export_plugin_description_file(nav2_core mpc_controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  add_executable(plugin_load_test test/plugin_load_test.cpp)
  target_compile_features(plugin_load_test PRIVATE cxx_std_17)
  target_compile_options(plugin_load_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  ament_target_dependencies(plugin_load_test nav2_core pluginlib)
  add_test(NAME pluginlib_load COMMAND plugin_load_test)
endif()

install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mpc_command_watchdog RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  arena_mpc_core geometry_msgs hunav_msgs nav2_core nav2_costmap_2d nav2_msgs nav_msgs
  pluginlib rclcpp rclcpp_lifecycle sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)
ament_package()
