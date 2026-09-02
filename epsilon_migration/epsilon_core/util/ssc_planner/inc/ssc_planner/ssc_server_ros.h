/**
 * @file ssc_server_ros.h
 * @author GW
 * @brief SSC ROS 服务端接口：封装在线重规划、控制发布与可视化
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `SscPlannerServer`，它是 `SscPlanner` 的 ROS 封装层，负责：
 * 1. 接收语义地图输入
 * 2. 驱动固定频率的规划循环
 * 3. 管理 executing trajectory / next trajectory 的滚动切换
 * 4. 发布控制信号与 RViz 可视化
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H_

#include <chrono>
#include <numeric>
#include <thread>

#include "common/basics/colormap.h"
#include "common/basics/tic_toc.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/trajectory/frenet_traj.h"
#include "common/visualization/common_visualization_util.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include "ros/ros.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ssc_planner/map_adapter.h"
#include "ssc_planner/ssc_planner.h"
#include "ssc_planner/ssc_visualizer.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {

/**
 * @brief SSC 规划 ROS 服务端
 *
 * **定位**：
 * 这是 `SscPlanner` 的在线运行壳层，不直接负责优化求解，
 * 而负责把 planner 放进一个持续运行的 receding-horizon 循环里。
 *
 * **核心职责**：
 * - 缓存语义地图输入
 * - 触发初始规划与滚动重规划
 * - 执行轨迹与下一段轨迹的切换
 * - 发布控制指令、语义地图可视化和 SSC 可视化
 */
class SscPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using FrenetTrajectory = common::FrenetTrajectory;

  struct Config {
    // 输入语义地图的缓冲队列容量。
    int kInputBufferSize{100};
  };

  // 使用默认工作频率构造 server。
  SscPlannerServer(ros::NodeHandle nh, int ego_id);

  // 使用给定工作频率构造 server。
  SscPlannerServer(ros::NodeHandle nh, double work_rate, int ego_id);

  // 推送最新语义地图到输入缓冲区。
  void PushSemanticMap(const SemanticMapManager &smm);

  // 初始化 planner、本地发布器与配置。
  void Init(const std::string &config_path);

  // 启动主规划线程。
  void Start();

 private:
  // 单个周期的 planning callback。
  void PlanCycleCallback();

  // 执行一次滚动重规划。
  void Replan();

  // 发布控制和可视化输出。
  void PublishData();

  // 主线程循环。
  void MainThread();

  // 过滤低速奇异状态。
  ErrorType FilterSingularityState(const vec_E<common::State> &hist,
                                   common::State *filter_state);

  Config config_;

  bool is_replan_on_ = false;
  bool is_map_updated_ = false;
  bool use_sim_state_ = true;
  std::unique_ptr<FrenetTrajectory> executing_traj_;
  std::unique_ptr<FrenetTrajectory> next_traj_;

  SscPlanner planner_;
  SscPlannerAdapter map_adapter_;

  TicToc time_profile_tool_;
  decimal_t global_init_stamp_{0.0};

  // ros related
  ros::NodeHandle nh_;
  decimal_t work_rate_ = 20.0;
  int ego_id_;

  bool require_intervention_signal_ = false;
  ros::Publisher ctrl_signal_pub_;
  ros::Publisher map_marker_pub_;
  ros::Publisher executing_traj_vis_pub_;

  // input buffer
  moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_;
  SemanticMapManager last_smm_;
  semantic_map_manager::Visualizer *p_smm_vis_{nullptr};

  SscVisualizer *p_ssc_vis_{nullptr};
  int last_trajmk_cnt_{0};

  vec_E<common::State> desired_state_hist_;
  vec_E<common::State> ctrl_state_hist_;
};

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_SERVER_ROS_H__
