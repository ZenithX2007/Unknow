cmake_minimum_required(VERSION 3.8)
project(vehicle_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ArenaInfo.msg"
  "msg/ArenaInfoDynamic.msg"
  "msg/ArenaInfoStatic.msg"
  "msg/Circle.msg"
  "msg/CircleObstacle.msg"
  "msg/ControlSignal.msg"
  "msg/FreeState.msg"
  "msg/Lane.msg"
  "msg/LaneNet.msg"
  "msg/ObstacleSet.msg"
  "msg/OccupancyGridFloat.msg"
  "msg/OccupancyGridUInt8.msg"
  "msg/PolygonObstacle.msg"
  "msg/PredictedState.msg"
  "msg/PredictedTrajectory.msg"
  "msg/PredictedTrajectoryArray.msg"
  "msg/State.msg"
  "msg/Vehicle.msg"
  "msg/VehicleParam.msg"
  "msg/VehicleSet.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
