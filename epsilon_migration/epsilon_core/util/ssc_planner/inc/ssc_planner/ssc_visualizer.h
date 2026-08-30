/**
 * @file ssc_visualizer.h
 * @author GW
 * @brief SSC 可视化器接口：发布 SSC 地图、走廊与轨迹的 RViz Marker
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 声明 `SscVisualizer`，用于把 `SscPlanner` 的关键中间结果映射到
 * `ssc_map` 坐标系下的可视化话题。
 */
#ifndef _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
#define _UTIL_SSC_PLANNER_INC_VISUALIZER_H_

#include <assert.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/primitive/frenet_primitive.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "ssc_planner/ssc_planner.h"

namespace planning {

/**
 * @brief SSC 可视化器
 *
 * **功能**：
 * 统一管理 SSC 相关可视化发布，包括：
 * - 3D 占据网格
 * - 自车 Frenet 状态
 * - 前向仿真轨迹
 * - 周边车预测轨迹
 * - 驾驶走廊
 * - QP Bézier 轨迹
 */
class SscVisualizer {
 public:
  // 根据 agent id 创建一组独立的话题发布器。
  SscVisualizer(ros::NodeHandle nh, int node_id);
  ~SscVisualizer() {}

  // 发布一次完整的 SSC 可视化结果。
  void VisualizeDataWithStamp(const ros::Time &stamp,
                              const SscPlanner &planner);

 private:
  // 可视化 3D SSC 占据网格。
  void VisualizeSscMap(const ros::Time &stamp, const SscMap *p_ssc_map);
  // 可视化自车 Frenet 轮廓和状态点。
  void VisualizeEgoVehicleInSscSpace(const ros::Time &stamp,
                                     const common::FsVehicle &fs_ego_vehicle);
  // 可视化自车前向仿真轨迹。
  void VisualizeForwardTrajectoriesInSscSpace(
      const ros::Time &stamp, const vec_E<vec_E<common::FsVehicle>> &trajs,
      const SscMap *p_ssc_map);
  // 可视化 QP 输出的 Bézier 轨迹。
  void VisualizeQpTrajs(const ros::Time &stamp,
                        const vec_E<common::BezierSpline<5, 2>> &trajs);
  // 可视化周边车预测轨迹。
  void VisualizeSurroundingVehicleTrajInSscSpace(
      const ros::Time &stamp,
      const vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> &trajs_set,
      const SscMap *p_ssc_map);
  // 可视化 driving corridor。
  void VisualizeCorridorsInSscSpace(
      const ros::Time &stamp, const vec_E<common::DrivingCorridor> corridor_vec,
      const SscMap *p_ssc_map);

  int last_traj_list_marker_cnt_ = 0;
  int last_surrounding_vehicle_marker_cnt_ = 0;

  ros::NodeHandle nh_;
  int node_id_;

  decimal_t start_time_;

  ros::Publisher ssc_map_pub_;
  ros::Publisher ego_vehicle_pub_;
  ros::Publisher forward_trajs_pub_;
  ros::Publisher sur_vehicle_trajs_pub_;
  ros::Publisher corridor_pub_;
  ros::Publisher qp_pub_;

  int last_corridor_mk_cnt = 0;
  int last_qp_traj_mk_cnt = 0;
  int last_sur_vehicle_traj_mk_cnt = 0;
  int last_forward_traj_mk_cnt = 0;
};  // SscVisualizer
}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_VISUALIZER_H_
