#ifndef _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__
#define _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__

/**
 * @file multimodal_forward.h
 * @author GW
 * @brief 多模态前向仿真参数工具：按激进度生成驾驶模型参数
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `MultiModalForward`，
 * 目前主要承担参数查表功能，把离散 aggressiveness level
 * 转换成 `OnLaneForwardSimulation::Param`。
 */

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"

#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

/**
 * @brief 多模态前向仿真参数查表工具
 */
class MultiModalForward {
 public:
  using Lane = common::Lane;
  using VehicleSet = common::VehicleSet;
  using GridMap = common::GridMapND<uint8_t, 2>;
  using State = common::State;
  typedef int AggressivenessLevel;

  static ErrorType ParamLookUp(const AggressivenessLevel& agg_level,
                               OnLaneForwardSimulation::Param* param) {
    switch (agg_level) {
      case 1:
        param->idm_param.kDesiredHeadwayTime = 2.0;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.0;
        param->steer_control_gain = 2.0;
        break;
      case 2:
        param->idm_param.kDesiredHeadwayTime = 1.7;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.67;
        param->steer_control_gain = 2.0;
        break;
      case 3:
        param->idm_param.kDesiredHeadwayTime = 1.5;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 4:
        param->idm_param.kDesiredHeadwayTime = 1.0;
        param->idm_param.kMinimumSpacing = 1.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 5:
        param->idm_param.kDesiredHeadwayTime = 0.5;
        param->idm_param.kMinimumSpacing = 1.0;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      default:
        assert(false);
        break;
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif
