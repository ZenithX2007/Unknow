/**
 * @file ssc_visualizer.cc
 * @author GW
 * @brief SSC 可视化器实现：将 SSC 地图、走廊与优化轨迹发布到 RViz
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件负责把 `SscPlanner` 运行过程中的关键中间结果映射到 `ssc_map`
 * 坐标系下的 RViz Marker：
 * 1. 三维占据网格（SSC map）
 * 2. 自车 Frenet 状态与轮廓
 * 3. 自车前向仿真轨迹
 * 4. 周边车预测轨迹
 * 5. 驾驶走廊（Driving Corridor）
 * 6. QP 优化得到的 Bézier 轨迹
 *
 * **坐标系说明**：
 * - x 轴：Frenet 纵向 `s`
 * - y 轴：Frenet 横向 `l`
 * - z 轴：相对时间 `t - start_time_`
 *
 * **对应文档**：
 * - `paper/SSC.md` 第 5 节：SSC 地图与走廊
 * - `paper/SSC.md` 第 6 节：QP 轨迹生成
 */

#include "ssc_planner/ssc_visualizer.h"

namespace planning {

/**
 * @brief 构造 SSC 可视化器
 * @param nh ROS 节点句柄
 * @param node_id 节点对应的 agent id
 *
 * **功能**：
 * 为不同类型的可视化结果创建独立的 MarkerArray 发布器。
 *
 * **发布话题**：
 * - `/vis/agent_x/ssc/map_vis`
 * - `/vis/agent_x/ssc/ego_fs_vis`
 * - `/vis/agent_x/ssc/forward_trajs_vis`
 * - `/vis/agent_x/ssc/sur_vehicle_trajs_vis`
 * - `/vis/agent_x/ssc/corridor_vis`
 * - `/vis/agent_x/ssc/qp_vis`
 */
SscVisualizer::SscVisualizer(ros::NodeHandle nh, int node_id)
    : nh_(nh), node_id_(node_id) {
  std::cout << "node_id_ = " << node_id_ << std::endl;

  std::string ssc_map_vis_topic = std::string("/vis/agent_") +
                                  std::to_string(node_id_) +
                                  std::string("/ssc/map_vis");
  std::string ego_vehicle_vis_topic = std::string("/vis/agent_") +
                                      std::to_string(node_id_) +
                                      std::string("/ssc/ego_fs_vis");
  std::string forward_trajs_vis_topic = std::string("/vis/agent_") +
                                        std::to_string(node_id_) +
                                        std::string("/ssc/forward_trajs_vis");
  std::string sur_vehicle_trajs_vis_topic =
      std::string("/vis/agent_") + std::to_string(node_id_) +
      std::string("/ssc/sur_vehicle_trajs_vis");
  std::string corridor_vis_topic = std::string("/vis/agent_") +
                                   std::to_string(node_id_) +
                                   std::string("/ssc/corridor_vis");
  std::string qp_vis_topic = std::string("/vis/agent_") +
                             std::to_string(node_id_) +
                             std::string("/ssc/qp_vis");

  ssc_map_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(ssc_map_vis_topic, 1);
  qp_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(qp_vis_topic, 1);
  ego_vehicle_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(ego_vehicle_vis_topic, 1);
  forward_trajs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      forward_trajs_vis_topic, 1);
  sur_vehicle_trajs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      sur_vehicle_trajs_vis_topic, 1);
  corridor_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(corridor_vis_topic, 1);
}

/**
 * @brief 发布一次完整的 SSC 可视化结果
 * @param stamp ROS 时间戳
 * @param planner 当前规划器对象
 *
 * **功能**：
 * 统一调度各类可视化子模块，把同一规划周期的中间结果成组发布。
 *
 * **发布内容**：
 * 1. SSC 占据网格
 * 2. 自车 Frenet 状态
 * 3. 前向仿真轨迹
 * 4. 周边车预测轨迹
 * 5. 驾驶走廊
 * 6. QP 轨迹
 *
 * **说明**：
 * - `start_time_` 取自规划器的 `time_origin()`，用于将绝对时间转换为相对时间高度
 */
void SscVisualizer::VisualizeDataWithStamp(const ros::Time &stamp,
                                           const SscPlanner &planner) {
  start_time_ = planner.time_origin();
  // std::cout << "[SscMapTime]Sys time stamp = " << stamp << std::endl;
  VisualizeSscMap(stamp, planner.p_ssc_map());
  VisualizeEgoVehicleInSscSpace(stamp, planner.fs_ego_vehicle());
  VisualizeForwardTrajectoriesInSscSpace(stamp, planner.forward_trajs_fs(),
                                         planner.p_ssc_map());
  VisualizeSurroundingVehicleTrajInSscSpace(
      stamp, planner.surround_forward_trajs_fs(), planner.p_ssc_map());
  VisualizeCorridorsInSscSpace(
      stamp, planner.p_ssc_map()->driving_corridor_vec(), planner.p_ssc_map());
  VisualizeQpTrajs(stamp, planner.qp_trajs());
}

/**
 * @brief 可视化 QP 优化后的 Bézier 轨迹
 * @param stamp ROS 时间戳
 * @param trajs Bézier 样条轨迹集合
 *
 * **功能**：
 * 对每条 QP 轨迹按时间采样，生成 `LINE_STRIP`，并绘制在 `s-l-t` 空间中。
 *
 * **显示语义**：
 * - x: 纵向 `s`
 * - y: 横向 `l`
 * - z: 相对时间
 *
 * **对应文档**：
 * - `paper/SSC.md` 第 6 节：分段 Bézier 轨迹优化结果
 */
void SscVisualizer::VisualizeQpTrajs(
    const ros::Time &stamp, const vec_E<common::BezierSpline<5, 2>> &trajs) {
  if (trajs.empty()) {
    printf("[SscQP]No valid qp trajs.\n");
    return;
  }
  int id = 0;
  visualization_msgs::MarkerArray traj_mk_arr;
  for (int i = 0; i < static_cast<int>(trajs.size()); i++) {
    visualization_msgs::Marker traj_mk;
    traj_mk.type = visualization_msgs::Marker::LINE_STRIP;
    traj_mk.action = visualization_msgs::Marker::MODIFY;
    traj_mk.id = id++;
    Vecf<2> pos;
    for (decimal_t t = trajs[i].begin(); t < trajs[i].end() + kEPS; t += 0.02) {
      if (trajs[i].evaluate(t, 0, &pos) == kSuccess) {
        geometry_msgs::Point pt;
        pt.x = pos[0];
        pt.y = pos[1];
        pt.z = t - start_time_;
        traj_mk.points.push_back(pt);
      }
    }
    common::VisualizationUtil::FillScaleColorInMarker(
        Vec3f(0.2, 0.2, 0.2), common::cmap.at("magenta"), &traj_mk);
    traj_mk_arr.markers.push_back(traj_mk);
  }

  int num_markers = static_cast<int>(traj_mk_arr.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("ssc_map"), last_qp_traj_mk_cnt, &traj_mk_arr);
  qp_pub_.publish(traj_mk_arr);
  last_qp_traj_mk_cnt = num_markers;
}

/**
 * @brief 可视化 SSC 三维占据网格
 * @param stamp ROS 时间戳
 * @param p_ssc_map SSC 地图对象
 *
 * **功能**：
 * 将 `SscMap` 中的 3D 占据网格渲染为体素列表，并叠加一个底部范围框。
 *
 * **显示内容**：
 * - 占据网格：静态/动态障碍物在 `s-l-t` 空间中的离散占据
 * - AABB 边框：帮助观察纵向范围和横向宽度
 *
 * **对应文档**：
 * - `paper/SSC.md` 第 5.1 节：类障碍物语义元素渲染到 slt 域
 */
void SscVisualizer::VisualizeSscMap(const ros::Time &stamp,
                                    const SscMap *p_ssc_map) {
  visualization_msgs::MarkerArray map_marker_arr;
  visualization_msgs::Marker map_marker;

  common::VisualizationUtil::GetRosMarkerCubeListUsingGripMap3D(
      p_ssc_map->p_3d_grid(), stamp, "ssc_map", Vec3f(0, 0, 0), &map_marker);

  auto origin = p_ssc_map->p_3d_grid()->origin();
  decimal_t s_len =
      p_ssc_map->config().map_resolution[0] * p_ssc_map->config().map_size[0];
  decimal_t x = s_len / 2 - p_ssc_map->config().s_back_len + origin[0];
  decimal_t y = 0;
  // decimal_t z = t_len / 2;
  std::array<decimal_t, 3> aabb_coord = {x, y, 0};
  std::array<decimal_t, 3> aabb_len = {s_len, 3.5, 0.01};
  common::AxisAlignedBoundingBoxND<3> map_aabb(aabb_coord, aabb_len);
  visualization_msgs::Marker map_aabb_marker;
  map_aabb_marker.header.frame_id = "ssc_map";
  map_aabb_marker.header.stamp = stamp;
  map_aabb_marker.id = 1;
  common::VisualizationUtil::GetRosMarkerCubeUsingAxisAlignedBoundingBox3D(
      map_aabb, common::ColorARGB(0.2, 1, 0, 1), &map_aabb_marker);

  map_marker_arr.markers.push_back(map_marker);
  map_marker_arr.markers.push_back(map_aabb_marker);
  ssc_map_pub_.publish(map_marker_arr);
}

/**
 * @brief 可视化自车在 SSC 空间中的状态
 * @param stamp ROS 时间戳
 * @param fs_ego_vehicle 自车 Frenet 车辆表示
 *
 * **功能**：
 * 显示自车轮廓投影和 Frenet 质点位置，帮助观察规划起点在 `s-l-t` 空间中的位置。
 *
 * **显示元素**：
 * - 车辆轮廓：由顶点连成的闭合折线
 * - Frenet 状态点：自车 `(s, l, t)` 位置的球体标记
 */
void SscVisualizer::VisualizeEgoVehicleInSscSpace(
    const ros::Time &stamp, const common::FsVehicle &fs_ego_vehicle) {
  if (fs_ego_vehicle.vertices.empty()) return;
  visualization_msgs::MarkerArray ego_vehicle_mks;

  visualization_msgs::Marker ego_contour_marker;
  common::ColorARGB color(0.8, 1.0, 0.0, 0.0);
  decimal_t dt = fs_ego_vehicle.frenet_state.time_stamp - start_time_;
  vec_E<Vec2f> contour = fs_ego_vehicle.vertices;
  contour.push_back(contour.front());
  common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
      contour, color, Vec3f(0.3, 0.3, 0.3), dt, 0, &ego_contour_marker);
  ego_contour_marker.header.frame_id = "ssc_map";
  ego_contour_marker.header.stamp = stamp;
  ego_vehicle_mks.markers.push_back(ego_contour_marker);

  visualization_msgs::Marker ego_fs_mk;
  common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
      Vec3f(fs_ego_vehicle.frenet_state.vec_s[0],
            fs_ego_vehicle.frenet_state.vec_dt[0], dt),
      common::cmap.at("red"), Vec3f(0.3, 0.3, 0.3), 1, &ego_fs_mk);
  ego_fs_mk.header.frame_id = "ssc_map";
  ego_fs_mk.header.stamp = stamp;
  ego_vehicle_mks.markers.push_back(ego_fs_mk);

  ego_vehicle_pub_.publish(ego_vehicle_mks);
}

/**
 * @brief 可视化自车前向仿真轨迹
 * @param stamp ROS 时间戳
 * @param trajs 每个候选行为对应的 Frenet 前向轨迹
 * @param p_ssc_map SSC 地图对象
 *
 * **功能**：
 * 将行为层或前向仿真层提供的候选轨迹投影到 `s-l-t` 空间，
 * 用于观察 SSC 种子与行为拓扑。
 *
 * **显示方式**：
 * - 每个离散状态显示车辆轮廓
 * - 同时用球体显示该时刻的 Frenet 质点
 * - 颜色随轨迹内时间步变化
 *
 * **对应文档**：
 * - `paper/SSC.md` 第 5.2 节：走廊生成的种子来自前向仿真轨迹
 */
void SscVisualizer::VisualizeForwardTrajectoriesInSscSpace(
    const ros::Time &stamp, const vec_E<vec_E<common::FsVehicle>> &trajs,
    const SscMap *p_ssc_map) {
  if (trajs.empty()) return;
  visualization_msgs::MarkerArray trajs_markers;
  int id_cnt = 0;
  //   common::ColorARGB color = common::cmap.at("gold");
  //   color.a = 0.4;
  for (int i = 0; i < static_cast<int>(trajs.size()); ++i) {
    if (trajs[i].empty()) continue;
    for (int k = 0; k < static_cast<int>(trajs[i].size()); ++k) {
      visualization_msgs::Marker vehicle_marker;
      vec_E<Vec2f> contour = trajs[i][k].vertices;
      bool is_valid = true;
      for (const auto v : contour) {
        if (v(0) <= 0) {
          is_valid = false;
          break;
        }
      }
      if (!is_valid) {
        continue;
      }
      common::ColorARGB color = common::GetJetColorByValue(
          static_cast<decimal_t>(k),
          static_cast<decimal_t>(trajs[i].size() - 1), 0.0);
      contour.push_back(contour.front());
      decimal_t dt = trajs[i][k].frenet_state.time_stamp - start_time_;
      common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
          contour, color, Vec3f(0.1, 0.1, 0.1), dt, id_cnt++, &vehicle_marker);
      trajs_markers.markers.push_back(vehicle_marker);

      visualization_msgs::Marker fs_mk;
      auto fs = trajs[i][k].frenet_state;
      decimal_t x = fs.vec_s[0];
      decimal_t y = fs.vec_dt[0];
      common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
          Vec3f(x, y, dt), common::cmap.at("cyan"), Vec3f(0.2, 0.2, 0.2),
          id_cnt++, &fs_mk);
      trajs_markers.markers.push_back(fs_mk);
    }
  }

  int num_markers = static_cast<int>(trajs_markers.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("ssc_map"), last_forward_traj_mk_cnt, &trajs_markers);
  forward_trajs_pub_.publish(trajs_markers);
  last_forward_traj_mk_cnt = num_markers;
}

/**
 * @brief 可视化周边车辆预测轨迹
 * @param stamp ROS 时间戳
 * @param trajs_set 不同行为下的周边车 Frenet 预测轨迹集合
 * @param p_ssc_map SSC 地图对象
 *
 * **功能**：
 * 显示周边车辆在 `s-l-t` 空间中的离散占位演化。
 *
 * **说明**：
 * - 当前实现只显示 `trajs_set.front()`，即默认取第一组行为对应的周边车轨迹
 * - 这是一种简化显示方式，便于观察主要动态障碍物走势
 *
 * **对应文档**：
 * - `paper/SSC.md` 第 5.1 节：动态障碍物由预测轨迹渲染而来
 */
void SscVisualizer::VisualizeSurroundingVehicleTrajInSscSpace(
    const ros::Time &stamp,
    const vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> &trajs_set,
    const SscMap *p_ssc_map) {
  visualization_msgs::MarkerArray trajs_markers;
  if (!trajs_set.empty()) {
    auto trajs = trajs_set.front();
    if (!trajs.empty()) {
      int id_cnt = 0;
      for (auto it = trajs.begin(); it != trajs.end(); ++it) {
        if (it->second.empty()) continue;
        for (int k = 0; k < static_cast<int>(it->second.size()); ++k) {
          visualization_msgs::Marker vehicle_marker;
          common::ColorARGB color = common::GetJetColorByValue(
              static_cast<decimal_t>(k),
              static_cast<decimal_t>(it->second.size() - 1), 0.0);
          vec_E<Vec2f> contour = it->second[k].vertices;
          bool is_valid = true;
          for (const auto v : contour) {
            if (v(0) <= 0) {
              is_valid = false;
              break;
            }
          }
          if (!is_valid) {
            continue;
          }
          contour.push_back(contour.front());
          decimal_t dt = it->second[k].frenet_state.time_stamp - start_time_;
          common::VisualizationUtil::
              GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
                  contour, color, Vec3f(0.1, 0.1, 0.1), dt, id_cnt++,
                  &vehicle_marker);
          vehicle_marker.header.frame_id = "ssc_map";
          vehicle_marker.header.stamp = stamp;
          trajs_markers.markers.push_back(vehicle_marker);
        }
      }
    }
  }

  int num_markers = static_cast<int>(trajs_markers.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      stamp, std::string("ssc_map"), last_sur_vehicle_traj_mk_cnt,
      &trajs_markers);
  sur_vehicle_trajs_pub_.publish(trajs_markers);
  last_sur_vehicle_traj_mk_cnt = num_markers;
}

/**
 * @brief 可视化驾驶走廊
 * @param stamp ROS 时间戳
 * @param corridor_vec 驾驶走廊集合
 * @param p_ssc_map SSC 地图对象
 *
 * **功能**：
 * 将 `DrivingCorridor` 中的种子点与膨胀后的时空立方体发布为 Marker。
 *
 * **显示元素**：
 * - Seed：走廊生成算法中的离散种子
 * - Cube：每个 driving cube 对应的时空包围盒
 *
 * **用途**：
 * - 直观检查“种子 -> 初始立方体 -> 膨胀走廊”的结果
 * - 对照 QP 轨迹是否始终位于走廊内部
 *
 * **对应文档**：
 * - `paper/SSC.md` 第 5.2 节：语义走廊生成
 */
void SscVisualizer::VisualizeCorridorsInSscSpace(
    const ros::Time &stamp, const vec_E<common::DrivingCorridor> corridor_vec,
    const SscMap *p_ssc_map) {
  if (corridor_vec.empty()) return;
  visualization_msgs::MarkerArray corridor_vec_marker;
  int id_cnt = 0;
  for (const auto &corridor : corridor_vec) {
    // if (corridor.is_valid) continue;
    int cube_cnt = 0;
    for (const auto &driving_cube : corridor.cubes) {
      // * Show seeds
      common::ColorARGB color = common::GetJetColorByValue(
          cube_cnt, corridor.cubes.size() - 1, -kEPS);
      for (const auto &seed : driving_cube.seeds) {
        visualization_msgs::Marker seed_marker;
        decimal_t s_x, s_y, s_z;
        p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(seed(0), 0,
                                                                     &s_x);
        p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(seed(1), 1,
                                                                     &s_y);
        p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(seed(2), 2,
                                                                     &s_z);
        common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
            Vec3f(s_x, s_y, s_z - start_time_), color, Vec3f(0.3, 0.3, 0.3),
            id_cnt++, &seed_marker);
        corridor_vec_marker.markers.push_back(seed_marker);
      }

      // * Show driving cube
      decimal_t x_max, x_min;
      decimal_t y_max, y_min;
      decimal_t z_max, z_min;
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.upper_bound[0], 0, &x_max);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.lower_bound[0], 0, &x_min);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.upper_bound[1], 1, &y_max);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.lower_bound[1], 1, &y_min);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.upper_bound[2], 2, &z_max);
      p_ssc_map->p_3d_grid()->GetGlobalMetricUsingCoordOnSingleDim(
          driving_cube.cube.lower_bound[2], 2, &z_min);
      decimal_t dx = x_max - x_min;
      decimal_t dy = y_max - y_min;
      decimal_t dz = z_max - z_min;
      decimal_t x = x_min + dx / 2.0;
      decimal_t y = y_min + dy / 2.0;
      decimal_t z = z_min - start_time_ + dz / 2.0;

      std::array<decimal_t, 3> aabb_coord = {x, y, z};
      std::array<decimal_t, 3> aabb_len = {dx, dy, dz};
      common::AxisAlignedBoundingBoxND<3> map_aabb(aabb_coord, aabb_len);
      visualization_msgs::Marker map_aabb_marker;
      map_aabb_marker.id = id_cnt++;
      common::VisualizationUtil::GetRosMarkerCubeUsingAxisAlignedBoundingBox3D(
          map_aabb, common::ColorARGB(0.15, 0.3, 1.0, 0.3), &map_aabb_marker);
      corridor_vec_marker.markers.push_back(map_aabb_marker);
      ++cube_cnt;
    }
  }

  int num_markers = static_cast<int>(corridor_vec_marker.markers.size());
  common::VisualizationUtil::FillHeaderIdInMarkerArray(
      ros::Time::now(), std::string("ssc_map"), last_corridor_mk_cnt,
      &corridor_vec_marker);
  corridor_pub_.publish(corridor_vec_marker);
  last_corridor_mk_cnt = num_markers;
}

}  // namespace planning
