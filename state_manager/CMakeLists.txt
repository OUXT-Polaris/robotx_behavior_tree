cmake_minimum_required(VERSION 3.8)
project(state_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(state_manager_component SHARED
  src/state_manager_component.cpp
)
target_compile_definitions(state_manager_component PRIVATE "STATE_MANAGER_COMPONENTDLL")

ament_auto_add_executable(state_manager_node
  src/state_manager_node.cpp
)
target_link_libraries(state_manager_node state_manager_component)
#rclcpp_components_register_nodes(navi_sim_component
#  "navi_sim::NaviSimComponent")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
