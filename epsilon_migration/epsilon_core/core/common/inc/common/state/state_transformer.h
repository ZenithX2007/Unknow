/**
 * @file state_transformer.h
 * @author GW
 * @brief 状态坐标变换器头文件：全局状态与 Frenet 状态双向转换接口
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `StateTransformer`，负责全局状态与 Frenet 状态互转。
 */
#ifndef _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__
#define _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__

#include "common/basics/basics.h"
#include "common/basics/config.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"

namespace common {
class StateTransformer {
 public:
  StateTransformer() {}
  StateTransformer(const Lane& lane) { lane_ = lane; }

  // 将 Frenet 状态恢复为世界坐标状态，供车辆模型和控制器执行。
  ErrorType GetStateFromFrenetState(const FrenetState& fs, State* s) const;

  /**
   * @brief 将世界坐标状态投影到参考车道的 Frenet 坐标系
   *
   * 投影依赖车道离散采样和局部拟合，因此位置可能存在约 1 cm
   * 的数值误差；该误差通常小于规划栅格分辨率，可用于在线规划。
   */
  ErrorType GetFrenetStateFromState(const State& s, FrenetState* fs) const;

  // 批量转换用于 SSC 地图和前向轨迹，保持输入顺序不变。
  ErrorType GetFrenetStateVectorFromStates(const vec_E<State> state_vec,
                                           vec_E<FrenetState>* fs_vec) const;

  // 将 Frenet 轨迹批量恢复到世界坐标，供可视化或执行层使用。
  ErrorType GetStateVectorFromFrenetStates(const vec_E<FrenetState>& fs_vec,
                                           vec_E<State>* state_vec) const;

  // 仅转换平面位置，不计算速度、曲率等动态量。
  ErrorType GetFrenetPointFromPoint(const Vec2f& s, Vec2f* fs) const;

  // 静态障碍物网格转换到 Frenet 平面时使用的批量接口。
  ErrorType GetFrenetPointVectorFromPoints(const vec_E<Vec2f>& s,
                                           vec_E<Vec2f>* fs) const;

  bool IsValid() const { return lane_.IsValid(); }

  void print() {}

 private:
  Lane lane_;
};

}  // namespace common

#endif
