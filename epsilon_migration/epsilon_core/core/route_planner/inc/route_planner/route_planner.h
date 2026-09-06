#ifndef _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_
#define _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_

/**
 * @file route_planner.h
 * @author GW
 * @brief 路径规划器接口：基于车道拓扑的导航路径扩展与导航状态维护
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `RoutePlanner`，
 * 负责在车道级拓扑图上维护一条供行为规划层使用的导航路径。
 *
 * **核心功能**：
 * 1. 根据当前最近车道构建导航路径
 * 2. 支持随机扩展或指定目标的路径模式
 * 3. 维护导航进度与任务状态
 * 4. 输出拼接后的导航参考车道
 */

#include <algorithm>
#include <memory>
#include <random>
#include <set>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"

namespace planning {

/**
 * @brief 车道级导航路径规划器
 */
class RoutePlanner : public Planner {
 public:
  // 随机扩展用于仿真场景；指定目标模式用于固定导航任务。
  enum NaviMode { kRandomExpansion, kAssignedTarget };
  // 导航状态机：生成路径、沿路径运行、到达路径尾部。
  enum NaviStatus { kReadyToGo, kInProgress, kFinished };

  std::string Name() override;

  ErrorType Init(const std::string config) override;

  ErrorType RunOnce() override;

  void set_navi_mode(const NaviMode& mode) { navi_mode_ = mode; };
  void set_ego_state(const common::State& state) { ego_state_ = state; }
  // 最近车道是拓扑搜索起点，不等于最终拼接后的连续参考车道。
  void set_nearest_lane_id(const int& id) { nearest_lane_id_ = id; }
  void set_assigned_navi_path(const std::vector<int>& path) {
    assigned_navi_path_ = path;
  }
  void set_lane_net(const common::LaneNet& lane_net) {
    lane_net_ = lane_net;
    if_get_lane_net_ = true;
  }

  // 车道 ID 序列供行为层进行导航和拓扑可达性判断。
  std::vector<int> navi_path() const { return navi_path_; }
  // 由车道 ID 序列拟合的连续几何参考线，供 Frenet 变换使用。
  common::Lane navi_lane() const { return navi_lane_; }

  bool if_get_lane_net() const { return if_get_lane_net_; }

  decimal_t navi_cur_arc_len() const { return navi_cur_arc_len_; }

 private:
  ErrorType GetChildLaneIds(const int lane_id, std::vector<int>* child_ids);

  // 拼接各段 lane points 并重新拟合为一条连续 Lane。
  ErrorType BuildNavigationLaneFromPath(const std::vector<int>& lane_ids);
  // 沿 child_id 随机扩展，生成满足最大长度的前向车道序列。
  ErrorType GetNaviPathByRandomExpansion();

  bool CheckIfArriveTargetLane();
  ErrorType CheckNaviProgress();

  ErrorType NaviLoopRandomExpansion();
  ErrorType NaviLoopAssignedTarget();

  common::LaneNet lane_net_;
  common::State ego_state_;

  int nearest_lane_id_;

  NaviStatus navi_status_ = kReadyToGo;
  NaviMode navi_mode_ = kRandomExpansion;

  bool if_restart_ = true;
  bool if_get_lane_net_ = false;

  // decimal_t navi_path_max_length_ = 1500;
  decimal_t navi_path_max_length_ = 200;
  decimal_t navi_start_arc_length_{0.0};
  // decimal_t navi_tail_remain_ = 200;
  decimal_t navi_path_length_{0.0};
  decimal_t navi_cur_arc_len_{0.0};

  std::vector<int> navi_path_;
  std::vector<int> assigned_navi_path_;
  common::Lane navi_lane_;

  std::random_device rd_gen_;
};

}  // namespace planning

#endif  //_CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_
