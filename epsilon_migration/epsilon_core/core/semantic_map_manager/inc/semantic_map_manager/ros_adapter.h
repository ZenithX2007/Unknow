#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_

/**
 * @file ros_adapter.h
 * @author GW
 * @brief 语义地图 ROS 适配器接口：订阅仿真消息并驱动 SemanticMapManager 更新
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `RosAdapter`，
 * 负责把仿真器发布的静态/动态 arena 消息转换成 `SemanticMapManager`
 * 可消费的数据流，并在地图更新后触发外部回调。
 */

#include <assert.h>

#include <functional>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "ros/ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "vehicle_msgs/decoder.h"

namespace semantic_map_manager {

/**
 * @brief SemanticMapManager 的 ROS 输入适配层
 */
class RosAdapter {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;

  RosAdapter() {}
  RosAdapter(ros::NodeHandle nh, SemanticMapManager* ptr_smm) : nh_(nh) {
    p_smm_ = ptr_smm;
    p_data_renderer_ = new DataRenderer(ptr_smm);
  }
  ~RosAdapter() {}

  // 地图完成一次静态/动态数据融合后，通知行为规划或可视化模块。
  void BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn);

  void Init();

 private:
  // 旧的一体化 arena 消息接口，保留兼容但不再作为主数据流。
  void ArenaInfoCallback(const vehicle_msgs::ArenaInfo::ConstPtr& msg);

  // 静态消息提供车道网和障碍物；通常只需在地图变化时更新。
  void ArenaInfoStaticCallback(
      const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg);
  // 动态消息提供车辆集合；收到后与最新静态快照融合并触发地图更新。
  void ArenaInfoDynamicCallback(
      const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg);

  ros::NodeHandle nh_;

  // communicate with phy simulator
  ros::Subscriber arena_info_sub_;
  ros::Subscriber arena_info_static_sub_;
  ros::Subscriber arena_info_dynamic_sub_;

  common::Vehicle ego_vehicle_;
  common::VehicleSet vehicle_set_;
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;

  DataRenderer* p_data_renderer_;
  SemanticMapManager* p_smm_;

  // 在得到静态地图前不处理动态车辆数据，避免构造不完整的语义快照。
  bool get_arena_info_static_ = false;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager&)> private_callback_fn_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
