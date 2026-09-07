cmake_minimum_required(VERSION 3.16)
project(arena_mpc_core VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(casadi 3.8.0 CONFIG REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/model.cpp
  src/solver.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC casadi::casadi)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  BUILD_RPATH_USE_ORIGIN ON
  INSTALL_RPATH "$ORIGIN"
)

add_executable(model_fixture test/model_fixture.cpp)
target_link_libraries(model_fixture PRIVATE ${PROJECT_NAME})
target_compile_options(model_fixture PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(solver_cases test/solver_cases.cpp)
target_link_libraries(solver_cases PRIVATE ${PROJECT_NAME})
target_compile_options(solver_cases PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(solver_fixture test/solver_fixture.cpp)
target_link_libraries(solver_fixture PRIVATE ${PROJECT_NAME})
target_compile_options(solver_fixture PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(mpc_benchmark benchmark/mpc_benchmark.cpp)
target_link_libraries(mpc_benchmark PRIVATE ${PROJECT_NAME})
target_compile_options(mpc_benchmark PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME model_fixture_cpp COMMAND model_fixture)
  add_test(NAME solver_cases_cpp COMMAND solver_cases)
  add_test(
    NAME model_python_reference
    COMMAND ${CMAKE_COMMAND} -E env
      MODEL_FIXTURE=$<TARGET_FILE:model_fixture>
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_reference.py
  )
  add_test(
    NAME solver_python_reference
    COMMAND ${CMAKE_COMMAND} -E env
      SOLVER_FIXTURE=$<TARGET_FILE:solver_fixture>
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/casadi-3.8.0-wheel
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_solver_reference.py
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS mpc_benchmark
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(casadi)
ament_export_include_directories(include)
ament_package()
