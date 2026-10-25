cmake_minimum_required(VERSION 3.22)
project(ego_gazebo_workspace LANGUAGES CXX)
find_package(Eigen3 REQUIRED)
add_executable(frame_tests tests/frame_tests.cpp)
target_compile_features(frame_tests PRIVATE cxx_std_17)
target_include_directories(frame_tests PRIVATE src/ego_gazebo/include)
target_link_libraries(frame_tests PRIVATE Eigen3::Eigen)
add_custom_target(ros_workspace ALL
  COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/build_workspace.sh"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  USES_TERMINAL)
include(CTest)
add_test(NAME frames COMMAND frame_tests)
add_test(NAME config_contracts COMMAND /usr/bin/python3 "${CMAKE_SOURCE_DIR}/tests/config_contracts.py")
add_test(NAME sitl_flight COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/test_runtime.sh")
add_test(NAME sitl_obstacle_route COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/test_runtime.sh" obstacle)
add_test(NAME map_collision COMMAND /usr/bin/python3 "${CMAKE_SOURCE_DIR}/tests/collision_probe.py")
set_tests_properties(map_collision PROPERTIES TIMEOUT 30 LABELS integration RUN_SERIAL TRUE)
set_tests_properties(sitl_flight sitl_obstacle_route PROPERTIES TIMEOUT 120 LABELS integration RUN_SERIAL TRUE)
