#ifndef _COMMON_INC_COMMON_INTERFACE_PLANNER_H__
#define _COMMON_INC_COMMON_INTERFACE_PLANNER_H__

/**
 * @file planner.h
 * @author GW
 * @brief 规划器抽象基类接口：统一行为/路径/轨迹规划模块的生命周期
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `planning::Planner` 抽象基类，
 * 统一约束各类规划模块的名称、初始化和单周期执行接口。
 */

#include <string>

#include "common/basics/basics.h"

namespace planning {
/**
 * @brief 规划器统一抽象基类
 */
class Planner {
 public:
  Planner() = default;

  virtual ~Planner() = default;

  virtual std::string Name() = 0;

  virtual ErrorType Init(const std::string config) = 0;

  virtual ErrorType RunOnce() = 0;
};

}  // namespace planning

#endif
