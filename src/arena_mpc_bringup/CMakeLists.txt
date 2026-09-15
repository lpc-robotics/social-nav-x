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
endif()

ament_package()
