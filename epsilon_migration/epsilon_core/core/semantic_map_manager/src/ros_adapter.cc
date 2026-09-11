/**
 * @file ros_adapter.cc
 * @author GW
 * @brief 语义地图 ROS 适配器实现：订阅 arena 消息并触发地图更新
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件实现 `RosAdapter`，
 * 负责订阅仿真器静态/动态 arena 话题、调用 `DataRenderer` 渲染，
 * 并在 `SemanticMapManager` 更新后把结果回调给规划层。
 */
#include "semantic_map_manager/ros_adapter.h"

namespace semantic_map_manager {

void RosAdapter::Init() {
  // 静态/动态话题拆分后，动态车辆可高频更新，车道和障碍物无需重复传输。
  {
    // arena_info_sub_ =
    //     nh_.subscribe("arena_info", 2, &RosAdapter::ArenaInfoCallback, this,
    //                   ros::TransportHints().tcpNoDelay());
    arena_info_static_sub_ = nh_.subscribe(
        "arena_info_static", 2, &RosAdapter::ArenaInfoStaticCallback, this,
        ros::TransportHints().tcpNoDelay());
    arena_info_dynamic_sub_ = nh_.subscribe(
        "arena_info_dynamic", 2, &RosAdapter::ArenaInfoDynamicCallback, this,
        ros::TransportHints().tcpNoDelay());
  }
}

// ! DEPRECATED (@lu.zhang)
void RosAdapter::ArenaInfoCallback(
    const vehicle_msgs::ArenaInfo::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
      *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
  p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                           obstacle_set_);
  if (has_callback_binded_) {
    private_callback_fn_(*p_smm_);
  }
}

void RosAdapter::ArenaInfoStaticCallback(
    const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  // 缓存静态环境，等待后续动态车辆消息组成同一轮完整输入。
  get_arena_info_static_ = true;
}

void RosAdapter::ArenaInfoDynamicCallback(
    const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(
      *msg, &time_stamp, &vehicle_set_);

  if (get_arena_info_static_) {
    // DataRenderer 在此处将真值几何信息转换为 SemanticMapManager 使用的
    // 障碍栅格、语义车道和关键车辆等中间表示。
    p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                             obstacle_set_);
    if (has_callback_binded_) {
      private_callback_fn_(*p_smm_);
    }
  }
}

void RosAdapter::BindMapUpdateCallback(
    std::function<int(const SemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager
