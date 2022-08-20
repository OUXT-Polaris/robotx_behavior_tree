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

#include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

#set(library_name ${PROJECT_NAME})

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

ament_auto_add_library(to_marker SHARED src/to_marker.cpp)

install(TARGETS
  ${plugin_libs}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
