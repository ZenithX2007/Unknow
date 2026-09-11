#ifndef _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__
#define _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__

#include "common/basics/basics.h"

namespace common {

class IntelligentDriverModel {
 public:
  struct State {
    decimal_t s{0.0};        // 自车纵向位置
    decimal_t v{0.0};        // 自车纵向速度
    decimal_t s_front{0.0};  // 前车纵向位置
    decimal_t v_front{0.0};  // 前车纵向速度

    State() {}
    State(const decimal_t &s_, const decimal_t &v_, const decimal_t &s_front_,
          const decimal_t &v_front_)
        : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
  };

  struct Param {
    decimal_t kDesiredVelocity = 0.0;                 // 期望自由流速度 v0
    decimal_t kVehicleLength = 5.0;                   // 前车等效长度
    decimal_t kMinimumSpacing = 2.0;                  // 最小静止间距 s0
    decimal_t kDesiredHeadwayTime = 1.0;              // 期望车头时距 T
    decimal_t kAcceleration = 2.0;                    // 最大舒适加速度 a
    decimal_t kComfortableBrakingDeceleration = 3.0;  // 舒适制动减速度 b
    decimal_t kHardBrakingDeceleration = 5.0;        // 硬制动上限
    int kExponent = 4;                                // 加速度指数 delta
  };

  // 标准 IDM：根据自车速度、期望速度和前车间距计算加速度。
  static ErrorType GetIdmDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);
  // 改进 IDM 变体，供需要不同跟驰响应的仿真模块使用。
  static ErrorType GetIIdmDesiredAcceleration(const Param &param,
                                              const State &cur_state,
                                              decimal_t *acc);
  // 工程实现使用的统一加速度入口。
  static ErrorType GetAccDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);
};

}  // namespace common

#endif
