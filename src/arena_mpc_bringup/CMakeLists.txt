cmake_minimum_required(VERSION 3.16)
project(arena_mpc_bringup VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/empty_human_states.py DESTINATION lib/${PROJECT_NAME})

ament_package()
