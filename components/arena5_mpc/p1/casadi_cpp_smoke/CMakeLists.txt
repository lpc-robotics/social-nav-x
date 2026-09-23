cmake_minimum_required(VERSION 3.16)
project(casadi_cpp_smoke LANGUAGES CXX)

find_package(casadi 3.8.0 CONFIG REQUIRED)

add_executable(casadi_cpp_smoke main.cpp)
target_compile_features(casadi_cpp_smoke PRIVATE cxx_std_17)
target_compile_options(casadi_cpp_smoke PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(casadi_cpp_smoke PRIVATE casadi::casadi)
set_target_properties(casadi_cpp_smoke PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")

install(TARGETS casadi_cpp_smoke RUNTIME DESTINATION bin)
