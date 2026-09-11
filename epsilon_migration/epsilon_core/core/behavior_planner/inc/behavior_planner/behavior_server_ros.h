#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS_H__
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_SERVER_ROS_H__

/**
 * @file behavior_server_ros.h
 * @author GW
 * @brief 行为规划 ROS 服务端接口：在线 MPDM 节点封装与系统集成层
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `BehaviorPlannerServer`，
 * 它负责把 `BehaviorPlanner` 包装成可持续运行的 ROS 在线规划节点。
 *
 * **核心职责**：
 * 1. 接收并缓存语义地图
 * 2. 启动固定频率后台规划线程
 * 3. 接收 joystick/HMI 输入
 * 4. 调用 `BehaviorPlanner` 完成一轮行为重规划
 * 5. 发布可视化并通过回调把结果传给下游模块
 */
#include "ros/ros.h"

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "behavior_planner/behavior_planner.h"
#include "behavior_planner/map_adapter.h"
#include "behavior_planner/visualizer.h"
#include "semantic_map_manager/semantic_map_manager.h"

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"

#include <sensor_msgs/Joy.h>
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {

/**
 * @brief MPDM 行为规划 ROS 服务端
 */
class BehaviorPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;

  struct Config {
    int kInputBufferSize{100};
  };

  BehaviorPlannerServer(ros::NodeHandle nh, int ego_id);

  BehaviorPlannerServer(ros::NodeHandle nh, double work_rate, int ego_id);

  void PushSemanticMap(const SemanticMapManager &smm);

  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);

  /**
   * @brief 设置自动驾驶等级
   */
  void set_autonomous_level(int level);

  /**
   * @brief 设置用户期望速度
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  void set_aggressive_level(int level);

  decimal_t user_desired_velocity() const;

  decimal_t reference_desired_velocity() const;

  // 允许接收 HMI/joystick 干预。
  void enable_hmi_interface();

  // 初始化行为规划器与可视化器。
  void Init();

  // 启动后台规划线程。
  void Start();

 private:
  void PlanCycleCallback();

  void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);

  void Replan();

  void PublishData();

  void MainThread();

  Config config_;

  BehaviorPlanner bp_;
  BehaviorPlannerMapAdapter map_adapter_;
  BehaviorPlannerVisualizer *p_visualizer_;

  TicToc time_profile_tool_;
  decimal_t global_init_stamp_{0.0};

  // ros related
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;

  double work_rate_;
  int ego_id_;

  // input buffer
  moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;

  bool is_hmi_enabled_ = false;
};

}  // namespace planning

#endif
