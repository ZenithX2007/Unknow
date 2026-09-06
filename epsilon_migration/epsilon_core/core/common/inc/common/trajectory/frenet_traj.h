/**
 * @file frenet_traj.h
 * @author GW
 * @brief Frenet 轨迹抽象头文件
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 Frenet 轨迹抽象接口。
 */
#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__

#include "common/basics/config.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/trajectory/trajectory.h"

namespace common {

class FrenetTrajectory : public Trajectory {
 public:
  virtual ~FrenetTrajectory() = default;
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;
  virtual ErrorType GetFrenetState(const decimal_t& t,
                                   FrenetState* fs) const = 0;
  virtual decimal_t begin() const = 0;
  virtual decimal_t end() const = 0;
  virtual bool IsValid() const = 0;
  virtual std::vector<decimal_t> variables()
      const = 0;  // return a copy of variables
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;
  // Frenet 查询用于碰撞检查和运动学约束，世界坐标查询用于车辆控制执行。
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const = 0;
};

}  // namespace common

#endif
