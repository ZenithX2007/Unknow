cmake_minimum_required(VERSION 3.8)
project(epsilon_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(epsilon_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)

add_executable(epsilon_integrated_planner_node
  src/epsilon_integrated_planner_node.cpp
)

add_executable(epsilon_scene_bridge_node
  src/epsilon_scene_bridge_node.cpp
)

add_executable(epsilon_cmd_vel_mux_node
  src/epsilon_cmd_vel_mux_node.cpp
)

ament_target_dependencies(epsilon_integrated_planner_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  vehicle_msgs
)

target_link_libraries(epsilon_integrated_planner_node
  epsilon_core::epsilon_core
)

ament_target_dependencies(epsilon_scene_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  vehicle_msgs
)

ament_target_dependencies(epsilon_cmd_vel_mux_node
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  epsilon_integrated_planner_node
  epsilon_scene_bridge_node
  epsilon_cmd_vel_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scenario_actor_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_tree config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
