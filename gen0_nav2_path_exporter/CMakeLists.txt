cmake_minimum_required(VERSION 3.8)
project(gen0_nav2_path_exporter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(gen0_nav2_path_exporter_bt_node SHARED
  src/publish_path_action.cpp
)

target_compile_definitions(gen0_nav2_path_exporter_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_compile_options(gen0_nav2_path_exporter_bt_node PRIVATE -Wall -Wextra -Wpedantic)

ament_target_dependencies(gen0_nav2_path_exporter_bt_node
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav_msgs
  rclcpp
)

install(TARGETS gen0_nav2_path_exporter_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav_msgs
  rclcpp
)

ament_package()
