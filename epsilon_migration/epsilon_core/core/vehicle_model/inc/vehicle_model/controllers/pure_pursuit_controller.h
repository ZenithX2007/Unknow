#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_PURE_PURSUIT_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_PURE_PURSUIT_H_

/**
 * @file pure_pursuit_controller.h
 * @author GW
 * @brief Pure Pursuit 控制器接口：根据预瞄点几何关系计算转角
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 */

#include "common/basics/basics.h"

namespace control {
/**
 * @brief Pure Pursuit 横向控制工具
 */
class PurePursuitControl {
 public:
  // 根据当前航向与预瞄点连线夹角，按自行车模型计算前轮目标转角。
  static ErrorType CalculateDesiredSteer(const decimal_t wheelbase_len,
                                         const decimal_t angle_diff,
                                         const decimal_t look_ahead_dist,
                                         decimal_t *steer);
};

}  // namespace control

#endif
