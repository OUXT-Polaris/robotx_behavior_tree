cmake_minimum_required(VERSION 3.5)
project(robotx_bt_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()


set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-fPIC)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_find_build_dependencies()

include_directories(${ZMQ_INCLUDE_DIR})
include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

ament_auto_add_library(${PROJECT_NAME}_component SHARED src/bt_planner_component.cpp)
ament_auto_add_library(calculation_action SHARED src/plugin/calculation_action.cpp)


ament_auto_add_executable(${PROJECT_NAME}_node src/bt_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)
target_link_libraries(${PROJECT_NAME}_node ${YAML_CPP_LIBRARIES} ${behaviortree_cpp_v3_LIBRARIES} ${ZMQ_LIBRARIES})

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
