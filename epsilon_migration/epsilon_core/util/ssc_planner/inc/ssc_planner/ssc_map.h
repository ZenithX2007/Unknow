/**
 * @file ssc_map.h
 * @author GW
 * @brief SSC 地图接口：维护 slt 三维占据网格与时空语义走廊
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `SscMap`，它是 `SscPlanner` 的地图后端，负责：
 * 1. 维护 slt 三维占据网格
 * 2. 把静态/动态障碍写入时空地图
 * 3. 围绕 seed 轨迹生成 driving corridor
 * 4. 将离散 corridor 转换为优化器使用的 metric cube
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
#define _UTIL_SSC_PLANNER_INC_SSC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "common/basics/semantics.h"
#include "common/state/state.h"

namespace planning {

using ObstacleMapType = uint8_t;
using SscMapDataType = uint8_t;

/**
 * @brief SSC 地图后端
 *
 * **定位**：
 * `SscMap` 不直接求轨迹，而是为后续 QP 优化构造时空可行域。
 *
 * **核心职责**：
 * - 构建 slt 三维占据图
 * - 管理 driving corridor
 * - 提供优化器需要的 final corridor metric 表达
 *
 * **坐标含义**：
 * - x -> `s`：纵向弧长
 * - y -> `d/l`：横向偏移
 * - z -> `t`：时间
 */
class SscMap {
 public:
  // SscMap 负责维护 SSC 规划器使用的三维时空栅格:
  //
  //   x -> s: 纵向弧长
  //   y -> d: 横向偏移
  //   z -> t: 时间
  //
  // 它的核心任务不是“直接求轨迹”，而是把障碍物和参考轨迹转换成
  // 优化器可消费的时空可行域。
  using GridMap3D = common::GridMapND<ObstacleMapType, 3>;

  struct Config {
    // 三维栅格尺寸，依次对应 s / d / t 三个维度的格子数量。
    std::array<int, 3> map_size = {{1000, 100, 81}};               // s, d, t
    // 三维栅格分辨率，依次对应纵向米、横向米、时间秒。
    std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.1}};  // m, m, s
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
    // 地图原点在 ego 后方预留的纵向长度。
    decimal_t s_back_len = 0.0;
    // 优化时附带给 corridor 的速度/加速度边界。
    decimal_t kMaxLongitudinalVel = 50.0;
    decimal_t kMinLongitudinalVel = 0.0;
    decimal_t kMaxLongitudinalAcc = 3.0;
    decimal_t kMaxLongitudinalDecel = -8.0;  // Avg. driver max
    decimal_t kMaxLateralVel = 3.0;
    decimal_t kMaxLateralAcc = 2.5;

    int kMaxNumOfGridAlongTime = 2;

    // 沿 +s, -s, +d, -d, +t, -t 的默认膨胀步长。
    // 实际实现里通常不会沿 -t 扩张。
    std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};

    void Print() {
      printf("\nSscMap Config:\n");
      printf(" -- map_size: [%d, %d, %d]\n", map_size[0], map_size[1],
             map_size[2]);
      printf(" -- map_resolution: [%lf, %lf, %lf]\n", map_resolution[0],
             map_resolution[1], map_resolution[2]);
      printf(" -- axis_name: [%s, %s, %s]\n", axis_name[0].c_str(),
             axis_name[1].c_str(), axis_name[2].c_str());
      printf(" -- s_back_len: %lf\n", s_back_len);
      printf(" -- kMaxLongitudinalVel: %lf\n", kMaxLongitudinalVel);
      printf(" -- kMinLongitudinalVel: %lf\n", kMinLongitudinalVel);
      printf(" -- kMaxLongitudinalAcc: %lf\n", kMaxLongitudinalAcc);
      printf(" -- kMaxLongitudinalDecel: %lf\n", kMaxLongitudinalDecel);
      printf(" -- kMaxLateralVel: %lf\n", kMaxLateralVel);
      printf(" -- kMaxLateralAcc: %lf\n", kMaxLateralAcc);
      printf(" -- kMaxNumOfGridAlongTime: %d\n", kMaxNumOfGridAlongTime);
      printf(" -- inflate_steps: [%d, %d, %d, %d, %d, %d]\n", inflate_steps[0],
             inflate_steps[1], inflate_steps[2], inflate_steps[3],
             inflate_steps[4], inflate_steps[5]);
    }
  };

  SscMap() {}
  SscMap(const Config &config);
  ~SscMap() {}

  // 返回原始 3D 占据图。
  GridMap3D *p_3d_grid() const { return p_3d_grid_; }
  // 返回障碍膨胀后的 3D 占据图。
  GridMap3D *p_3d_inflated_grid() const { return p_3d_inflated_grid_; }

  // 返回地图配置。
  Config config() const { return config_; }

  // 返回离散 driving corridor。
  vec_E<common::DrivingCorridor> driving_corridor_vec() const {
    return driving_corridor_vec_;
  }
  // 返回转换成 metric cube 的最终 corridor。
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec()
      const {
    return final_corridor_vec_;
  };

  // 返回每个 corridor 是否有效。
  std::vector<int> if_corridor_valid() const { return if_corridor_valid_; }

  // 设置规划起点时间。
  void set_start_time(const decimal_t &t) { start_time_ = t; }
  // 设置规划起点 Frenet 状态。
  void set_initial_fs(const common::FrenetState &fs) { initial_fs_ = fs; }

  // 按当前规划起点更新 3D 地图原点。
  void UpdateMapOrigin(const common::FrenetState &ori_fs);

  // 由静态障碍和动态障碍预测构建 3D 时空占据图。
  ErrorType ConstructSscMap(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs,
      const vec_E<Vec2f> &obstacle_grids);

  // 以车辆尺寸为模板，对障碍栅格做几何膨胀。
  ErrorType InflateObstacleGrid(const common::VehicleParam &param);

  // 围绕参考 Frenet 轨迹 seed 构建离散 corridor。
  ErrorType ConstructCorridorUsingInitialTrajectory(
      GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs);

  ErrorType ClearGridMap();

  ErrorType ClearDrivingCorridor();

  // 将离散 grid-index cube 转成优化器使用的 metric cube。
  ErrorType GetFinalGlobalMetricCubesList();

  // 以当前 Frenet 初始状态重置整张 SSC 地图。
  ErrorType ResetSscMap(const common::FrenetState &ini_frenet_state);

 private:
  // 判断整个 cube 是否处于空闲区域。
  bool CheckIfCubeIsFree(GridMap3D *p_grid,
                         const common::AxisAlignedCubeNd<int, 3> &cube) const;

  // 判断某个待扩张平面是否完全空闲。
  bool CheckIfPlaneIsFreeOnXAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfPlaneIsFreeOnYAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfPlaneIsFreeOnZAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfCubeContainsSeed(const common::AxisAlignedCubeNd<int, 3> &cube_a,
                               const Vec3i &seed) const;

  // 用相邻两个 seed 初始化最小包围 cube。
  ErrorType GetInitialCubeUsingSeed(
      const Vec3i &seed_0, const Vec3i &seed_1,
      common::AxisAlignedCubeNd<int, 3> *cube) const;

  // 统计在给定时间跨度内，corridor 能覆盖到哪些 cube。
  ErrorType GetTimeCoveredCubeIndices(const common::DrivingCorridor *p_corridor,
                                      const int &start_id, const int &dir,
                                      const int &t_trans,
                                      std::vector<int> *idx_list) const;

  // 让相邻 cube 之间的连接更平滑、减少狭窄瓶颈。
  ErrorType CorridorRelaxation(GridMap3D *p_grid,
                               common::DrivingCorridor *p_corridor);

  // 沿多个轴向迭代膨胀 cube，直到碰撞或达到运动学边界。
  ErrorType InflateCubeIn3dGrid(GridMap3D *p_grid,
                                const std::array<bool, 6> &dir_disabled,
                                const std::array<int, 6> &dir_step,
                                common::AxisAlignedCubeNd<int, 3> *cube);

  // 配置本轮 cube 允许向哪些方向膨胀。
  ErrorType GetInflationDirections(const bool &if_first_cube,
                                   std::array<bool, 6> *dirs_disabled);

  // 以下函数负责沿不同坐标轴方向逐层扩张 cube。
  bool InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);

  // 将 Frenet 平面中的静态障碍点复制到所有时间层。
  ErrorType FillStaticPart(const vec_E<Vec2f> &obs_grid_fs);

  // 将所有周围车预测写入动态障碍层。
  ErrorType FillDynamicPart(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs);

  // 在单个时间层上用 polygon rasterization 填入车辆占据。
  ErrorType FillMapWithFsVehicleTraj(const vec_E<common::FsVehicle> traj);

  // 原始 3D 占据图和经膨胀后的占据图。
  common::GridMapND<SscMapDataType, 3> *p_3d_grid_;
  common::GridMapND<SscMapDataType, 3> *p_3d_inflated_grid_;

  std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;

  Config config_;

  // 当前规划轮次的起始时间与 Frenet 初始状态。
  decimal_t start_time_;

  common::FrenetState initial_fs_;

  bool map_valid_ = false;

  // 中间 corridor 表达和最终给优化器的 corridor 表达。
  vec_E<common::DrivingCorridor> driving_corridor_vec_;

  std::vector<int> if_corridor_valid_;
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec_;
};

}  // namespace planning
#endif  // _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
