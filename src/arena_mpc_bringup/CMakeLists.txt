cmake_minimum_required(VERSION 3.16)
project(arena_mpc_bringup VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/empty_human_states.py
    scripts/mpc_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    mpc_visualizer_unit
    test/test_mpc_visualizer.py
    ENV
      MPC_VISUALIZER_PATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts/mpc_visualizer.py
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    nav2_overrides_contract
    test/test_nav2_overrides.py
    ENV
      MPC_NAV2_OVERRIDES_PATH=${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_overrides.yaml
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    depth_clearing_geometry
    test/test_depth_clearing_geometry.py
    ENV
      MPC_DEPTH_CLEARING_RUNTIME=${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/isaac_python
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
