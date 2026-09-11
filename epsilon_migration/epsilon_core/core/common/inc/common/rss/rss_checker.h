#ifndef _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__
#define _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__

/**
 * @file rss_checker.h
 * @author GW
 * @brief RSS 安全检查器接口：纵向/横向安全距离与安全速度计算
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `common::RssChecker`，
 * 用于根据 RSS（Responsibility-Sensitive Safety）规则计算安全距离、
 * 安全速度区间以及双车安全性判定。
 */

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"

namespace common {

/**
 * @brief RSS 安全检查工具类
 */
class RssChecker {
 public:
  // Front/Rear 表示另一辆车相对 ego 的纵向位置；Left/Right 表示横向位置。
  enum LongitudinalDirection { Front = 0, Rear };
  enum LateralDirection { Left = 0, Right };
  enum class LongitudinalViolateType { Legal = 0, TooFast, TooSlow };

  struct RssConfig {
    decimal_t response_time = 0.1;                 // 反应阶段持续时间
    decimal_t longitudinal_acc_max = 2.0;          // 反应阶段允许的最大纵向加速
    decimal_t longitudinal_brake_min = 4.0;        // ego 保证可实现的最小制动能力
    decimal_t longitudinal_brake_max = 5.0;        // 对方可能施加的最大制动能力
    decimal_t lateral_acc_max = 1.0;               // 反应阶段最大横向加速
    decimal_t lateral_brake_min = 1.0;             // ego 最小横向制动能力
    decimal_t lateral_brake_max = 1.0;             // 对方最大横向制动能力
    decimal_t lateral_miu = 0.5;                   // 额外横向安全裕量
    RssConfig() {}
    RssConfig(const decimal_t _response_time,
              const decimal_t _longitudinal_acc_max,
              const decimal_t _longitudinal_brake_min,
              const decimal_t _longitudinal_brake_max,
              const decimal_t _lateral_acc_max,
              const decimal_t _lateral_brake_min,
              const decimal_t _lateral_brake_max, const decimal_t _lateral_miu)
        : response_time(_response_time),
          longitudinal_acc_max(_longitudinal_acc_max),
          longitudinal_brake_min(_longitudinal_brake_min),
          longitudinal_brake_max(_longitudinal_brake_max),
          lateral_acc_max(_lateral_acc_max),
          lateral_brake_min(_lateral_brake_min),
          lateral_brake_max(_lateral_brake_max),
          lateral_miu(_lateral_miu) {}
  };

  // 计算在最不利反应/制动假设下所需的纵向净间距。
  static ErrorType CalculateSafeLongitudinalDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LongitudinalDirection& direction, const RssConfig& config,
      decimal_t* distance);

  // 计算横向相对运动下所需的最小横向净间距。
  static ErrorType CalculateSafeLateralDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LateralDirection& direction, const RssConfig& config,
      decimal_t* distance);

  static ErrorType CalculateRssSafeDistances(
      const std::vector<decimal_t>& ego_vels,
      const std::vector<decimal_t>& other_vels,
      const LongitudinalDirection& long_direct,
      const LateralDirection& lat_direct, const RssConfig& config,
      std::vector<decimal_t>* safe_distances);

  // 在 Frenet s-d 平面内同时比较纵向和横向安全距离。
  static ErrorType RssCheck(const FrenetState& ego_fs,
                            const FrenetState& other_fs,
                            const RssConfig& config, bool* is_safe);

  // 将车辆投影到同一参考线后检查 RSS，并给出允许的 ego 速度区间。
  static ErrorType RssCheck(const Vehicle& ego_vehicle,
                            const Vehicle& other_vehicle, const StateTransformer& stf,
                            const RssConfig& config, bool* is_safe,
                            LongitudinalViolateType* lon_type,
                            decimal_t* rss_vel_low, decimal_t* rss_vel_up);

  // 给定已有纵向净间距，反解 ego 合法的纵向速度范围。
  static ErrorType CalculateSafeLongitudinalVelocity(
      const decimal_t other_vel, const LongitudinalDirection& direction,
      const decimal_t& lon_distance_abs, const RssConfig& config,
      decimal_t* ego_vel_low, decimal_t* ego_vel_upp);
};

}  // namespace common

#endif
