#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_

/**
 * @file eudm_itf.h
 * @author GW
 * @brief EUDM 任务级接口定义：外部 lane change 约束与行为任务输入
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 */

#include <string>
#include <unordered_map>
#include <vector>

namespace planning {
namespace eudm {

/**
 * @brief 换道相关的外部约束与建议信号
 *
 * **作用**：
 * 由更上层模块注入，用于影响 EUDM 的横向行为选择。
 *
 * **包含两类信息**：
 * 1. **禁止/不安全信号**：
 *    - 禁止左/右换道
 *    - 左/右方向被占道判为不安全
 *    - 左/右侧是否为实线
 * 2. **推荐信号**：
 *    - 推荐左/右换道
 */
struct LaneChangeInfo {
  // 换道相关的外部约束和推荐信号。
  bool forbid_lane_change_left = false;
  bool forbid_lane_change_right = false;
  bool lane_change_left_unsafe_by_occu = false;
  bool lane_change_right_unsafe_by_occu = false;
  bool left_solid_lane = false;
  bool right_solid_lane = false;
  bool recommend_lc_left = false;
  bool recommend_lc_right = false;
};

/**
 * @brief EUDM 每轮接收的任务级输入
 *
 * **字段含义**：
 * - `is_under_ctrl`：当前是否处于自动驾驶控制模式
 * - `user_desired_vel`：用户或上层给出的期望速度
 * - `user_perferred_behavior`：用户显式横向偏好
 * - `lc_info`：换道相关的外部约束与推荐
 *
 * **说明**：
 * `user_perferred_behavior` 采用简化编码：
 * - `1`：请求右换道
 * - `-1`：请求左换道
 * - `0`：无显式换道请求
 */
struct Task {
  // 行为规划层每轮接收的任务级输入。
  bool is_under_ctrl = false;
  double user_desired_vel;
  // 约定:
  //  1  -> 右换道请求
  // -1  -> 左换道请求
  //  0  -> 无显式换道请求
  int user_perferred_behavior = 0;
  LaneChangeInfo lc_info;
};

}  // namespace eudm
}  // namespace planning

#endif
