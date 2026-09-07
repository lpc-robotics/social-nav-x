cmake_minimum_required(VERSION 3.16)
project(arena_mpc_core_consumer LANGUAGES CXX)

find_package(arena_mpc_core CONFIG REQUIRED)

add_executable(arena_mpc_core_consumer main.cpp)
target_compile_features(arena_mpc_core_consumer PRIVATE cxx_std_17)
target_compile_options(arena_mpc_core_consumer PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(arena_mpc_core_consumer PRIVATE arena_mpc_core::arena_mpc_core)
set_target_properties(arena_mpc_core_consumer PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
