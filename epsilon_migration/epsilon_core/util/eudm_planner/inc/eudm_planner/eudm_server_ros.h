#ifndef _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#define _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__

/**
 * @file eudm_server_ros.h
 * @author GW
 * @brief EUDM ROS 服务端接口：在线行为规划线程、HMI 输入与系统集成封装
 * @version 0.1
 * @date 2019-07-07
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `EudmPlannerServer`，
 * 它是 EUDM 行为规划器在 ROS 系统中的在线封装层。
 *
 * **核心职责**：
 * 1. 接收语义地图输入并维护缓冲区
 * 2. 维护固定频率后台规划线程
 * 3. 接收 joystick/HMI 任务输入并转为行为约束
 * 4. 调用 `EudmManager` 完成在线重规划
 * 5. 将规划结果回写到语义地图并触发回调/可视化
 */
#include <sensor_msgs/Joy.h>

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "eudm_planner/visualizer.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include "ros/ros.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {

/**
 * @brief EUDM ROS 在线服务端
 *
 * 这是 `EudmManager` 的系统集成包装器，负责把：
 * `SemanticMapManager + 用户输入 + 后台规划线程 + 可视化`
 * 组织成一个持续运行的在线行为规划节点。
 */
class EudmPlannerServer {
 public:
  // EUDM 的 ROS 在线封装层:
  // 接收语义地图、维护后台规划线程、接收 joystick/HMI 输入、
  // 调用 manager，并把行为结果写回语义地图。
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  struct Config {
    // 输入缓冲区长度。规划周期内通常只需要消费最新一帧语义地图。
    int kInputBufferSize{100};
  };

  // 使用默认 20Hz 工作频率构造在线规划节点。
  EudmPlannerServer(ros::NodeHandle nh, int ego_id);

  // 使用指定工作频率构造在线规划节点。
  EudmPlannerServer(ros::NodeHandle nh, double work_rate, int ego_id);

  // 把新的语义地图快照压入输入缓冲区。
  void PushSemanticMap(const SemanticMapManager &smm);

  // 绑定一个规划完成后的回调，便于和外部系统串联。
  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);

  /**
   * @brief 设置用户期望速度
   * @param desired_vel 用户给定的目标速度
   *
   * 该值会在下一轮 `PlanCycleCallback()` 中写入 `task_`，
   * 并进一步传给 `EudmManager` / `EudmPlanner`。
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  decimal_t user_desired_velocity() const;

  // 初始化 manager、订阅器和可视化器。
  void Init(const std::string &bp_config_path);

  // 启动后台规划线程。
  void Start();

 private:
  // 单个规划周期执行入口。
  void PlanCycleCallback();

  void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);

  void PublishData();

  // 固定频率后台工作线程。
  void MainThread();

  ErrorType GetCorrespondingActionInActionSequence(
      const decimal_t &t, const std::vector<DcpAction> &action_seq,
      DcpAction *a) const;

  Config config_;

  // manager 持有 planner 本体与任务级上下文。
  EudmManager bp_manager_;
  EudmPlannerVisualizer *p_visualizer_;

  // 当前消费到的最新语义地图快照。
  SemanticMapManager smm_;

  // 当前 HMI / joystick 任务输入。
  planning::eudm::Task task_;
  bool use_sim_state_ = true;
  // ros related
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;

  double work_rate_{20.0};
  int ego_id_;

  // 输入缓冲区。旧帧可被更新帧覆盖，保证规划尽可能使用最新环境。
  moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;
};

}  // namespace planning

#endif
