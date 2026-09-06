#ifndef _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_
#define _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_

/**
 * @file onlane_fs_predictor.h
 * @author GW
 * @brief 基于车道前向仿真的开环轨迹预测器接口
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `OnLaneFsPredictor`，
 * 它基于 `OnLaneForwardSimulation` 在给定参考车道上对单车做开环轨迹预测，
 * 供语义地图层或高层行为模块使用。
 */

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/**
 * @brief 基于单车道前向仿真的开环预测器
 */
class OnLaneFsPredictor {
 public:
  using Lane = common::Lane;
  using State = common::State;
  using VehicleControlSignal = common::VehicleControlSignal;
  using Vehicle = common::Vehicle;

  OnLaneFsPredictor() {}
  ~OnLaneFsPredictor();

  static ErrorType GetPredictedTrajectory(const Lane& lane,
                                          const Vehicle& vehicle,
                                          const decimal_t& t_pred,
                                          const decimal_t& t_step,
                                          vec_E<State>* pred_states) {
    // 这是地图层使用的开环基线预测：车辆以当前速度为期望速度，
    // 沿已知参考车道前推，不模拟其对自车决策的交互响应。
    pred_states->clear();
    int num_step = std::round(t_pred / t_step);
    State desired_state;
    decimal_t desired_vel = vehicle.state().velocity;
    planning::OnLaneForwardSimulation::Param sim_param;
    sim_param.idm_param.kDesiredVelocity = desired_vel;
    // 输出包含当前观测状态，便于调用方按时间索引做碰撞查询。
    pred_states->push_back(vehicle.state());
    common::Vehicle v_in = vehicle;
    common::StateTransformer stf = common::StateTransformer(lane);
    for (int i = 0; i < num_step; ++i) {
      if (lane.IsValid()) {
        // 有有效车道时，Pure Pursuit + IDM 使预测沿车道中心线推进。
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                stf, v_in, common::Vehicle(), t_step, sim_param,
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      } else {
        // 未匹配到车道时退化为保持当前转角/速度的运动学外推，
        // 避免预测模块因局部地图缺失而中断。
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                desired_vel, v_in, t_step,
                planning::OnLaneForwardSimulation::Param(),
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      }
      pred_states->push_back(desired_state);
      v_in.set_state(desired_state);
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif  // _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_
