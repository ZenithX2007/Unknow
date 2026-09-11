#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__

/**
 * @file trajectory.h
 * @author GW
 * @brief 轨迹抽象基类接口：统一连续轨迹对象的查询与优化变量访问
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `common::Trajectory` 抽象接口，
 * 统一描述连续轨迹对象的时间域查询、有效性判断和优化变量访问方式。
 */

#include "common/basics/config.h"
#include "common/state/state.h"

namespace common {

/**
 * @brief 连续轨迹对象统一抽象
 */
class Trajectory {
 public:
  virtual ~Trajectory() = default;
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;
  virtual decimal_t begin() const = 0;
  virtual decimal_t end() const = 0;
  virtual bool IsValid() const = 0;
  // optimization-related interface
  virtual std::vector<decimal_t> variables() const = 0;  // return a copy of variables
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;
};

}  // namespace common

#endif
