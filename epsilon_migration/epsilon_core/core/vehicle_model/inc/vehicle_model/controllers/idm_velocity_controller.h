#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_

/**
 * @file idm_velocity_controller.h
 * @author GW
 * @brief IDM 速度控制器接口：根据跟驰关系计算下一时刻期望速度
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 */

#include "common/basics/basics.h"

#include "vehicle_model/idm_model.h"

namespace control {

/**
 * @brief IDM 速度控制工具
 */
class IntelligentVelocityControl {
 public:
  // 将 IDM 期望加速度积分一个 dt，输出下一时刻的纵向速度。
  // s 和 s_front 使用同一条参考线上的纵向坐标。
  static ErrorType CalculateDesiredVelocity(
      const simulator::IntelligentDriverModel::Param& param, const decimal_t s,
      const decimal_t s_front, const decimal_t v, const decimal_t v_front,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif
