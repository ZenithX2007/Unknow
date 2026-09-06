#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_

/**
 * @file ctx_idm_velocity_controller.h
 * @author GW
 * @brief 上下文增强 IDM 速度控制器接口
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 */

#include "common/basics/basics.h"
#include "vehicle_model/ctx_idm_model.h"

namespace control {

/**
 * @brief Context-aware IDM 速度控制工具
 */
class ContextIntelligentVelocityControl {
 public:
  // 在普通跟驰约束之外，增加对目标状态 (s_target, v_target) 的跟踪，
  // 主要用于换道时驶入目标 gap。
  static ErrorType CalculateDesiredVelocity(
      const common::IntelligentDriverModel::Param& idm_param,
      const simulator::ContextIntelligentDriverModel::CtxParam& ctx_param,
      const decimal_t s, const decimal_t s_front, const decimal_t s_target,
      const decimal_t v, const decimal_t v_front, const decimal_t v_target,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif  //_CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_
