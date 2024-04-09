cmake_minimum_required(VERSION 3.5)
project(robotx_behavior_tree)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include ${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(example_action SHARED plugins/action/example_action.cpp)
target_compile_definitions(example_action PRIVATE BT_PLUGIN_EXPORT)

ament_auto_add_library(publish_action SHARED plugins/action/publish_action.cpp)
target_compile_definitions(publish_action PRIVATE BT_PLUGIN_EXPORT)

ament_auto_add_library(wait_action SHARED plugins/action/wait_action.cpp)
target_compile_definitions(wait_action PRIVATE BT_PLUGIN_EXPORT)

ament_auto_add_library(move_goal_action SHARED plugins/action/move_goal_action.cpp)
target_compile_definitions(move_goal_action PRIVATE BT_PLUGIN_EXPORT)

ament_auto_add_library(set_turn_action SHARED plugins/action/set_turn_action.cpp)
target_compile_definitions(set_turn_action PRIVATE BT_PLUGIN_EXPORT)

ament_auto_add_library(move_to_gate_action SHARED plugins/action/move_to_gate_action.cpp)
target_compile_definitions(move_to_gate_action PRIVATE BT_PLUGIN_EXPORT)

ament_auto_add_library(move_to_front_pose_of_object SHARED plugins/action/move_to_front_pose_of_object.cpp)
target_compile_definitions(move_to_front_pose_of_object PRIVATE BT_PLUGIN_EXPORT)

ament_auto_add_library(to_marker SHARED src/to_marker.cpp)

install(TARGETS
  ${plugin_libs}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

option(DEVELOP "If true, install 3d models for desktop development" OFF)

if(DEVELOP)
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_action_node test/src/test_action_node.cpp)
endif()

ament_auto_package()
