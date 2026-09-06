#ifndef _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_
#define _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_

/**
 * @file pid_controller.h
 * @author GW
 * @brief PID 控制器接口：用于标量目标跟踪的通用控制工具
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 */

#include "common/basics/basics.h"

#include <deque>

namespace control {

/**
 * @brief 通用 PID 控制器
 */
class PIDControl {
 public:
  struct ControlParam {
    decimal_t kP;  // 当前误差比例项
    decimal_t kI;  // 历史误差积分项
    decimal_t kD;  // 误差变化率微分项
    ControlParam() : kP(1.0), kI(1.0), kD(0.5) {}
    ControlParam(const decimal_t p, const decimal_t i, const decimal_t d)
        : kP(p), kI(i), kD(d) {}
  };

  PIDControl(const ControlParam& param);
  PIDControl(const ControlParam& param, const decimal_t dt);
  // 根据目标值与实际值输出一个标量控制量。
  decimal_t CalculatePIDControl(const decimal_t desired_state,
                                const decimal_t true_state);

 private:
  ControlParam param_;
  decimal_t dt_;
  int max_history_len_;
  std::deque<decimal_t> error_hist_;
};

}  // namespace control

#endif
