/**
 * @file dcp_tree.h
 * @author GW
 * @brief DCP-Tree 接口：EUDM 中用于动作空间引导分支的语义策略树
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_

#include <map>
#include <memory>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

/**
 * @brief DCP-Tree（领域特定闭环策略树）
 *
 * **定位**：
 * 这是 EUDM 在动作空间中的引导式分支结构，对应论文第 4.2 节。
 *
 * **核心思想**：
 * - 每条根到叶路径代表一条待评估的语义动作序列
 * - 在单个规划周期内，每条序列最多只允许一次动作变化
 * - 因此动作序列数量随树高近似线性增长，而不是指数增长
 *
 * **作用**：
 * 1. 为 `EudmPlanner` 生成有限数量的候选动作脚本
 * 2. 保留长期决策能力，同时控制动作空间复杂度
 */
class DcpTree {
 public:
  // DCP 树是论文中 guided action branching 的离散策略骨架。
  // 每一条根到叶路径都代表一条待评估的语义动作序列。
  using LateralBehavior = common::LateralBehavior;

  enum class DcpLonAction {
    // 纵向语义动作，会在后续仿真中映射成不同的纵向控制风格。
    kMaintain = 0,
    kAccelerate,
    kDecelerate,
    MAX_COUNT = 3
  };

  enum class DcpLatAction {
    // 横向语义动作，对应车道保持 / 左变道 / 右变道。
    kLaneKeeping = 0,
    kLaneChangeLeft,
    kLaneChangeRight,
    MAX_COUNT = 3
  };

  struct DcpAction {
    // 一个离散语义动作节点，由横纵向动作和该层持续时长组成。
    DcpLonAction lon = DcpLonAction::kMaintain;
    DcpLatAction lat = DcpLatAction::kLaneKeeping;

    decimal_t t = 0.0;

    friend std::ostream& operator<<(std::ostream& os, const DcpAction& action) {
      os << "(lon: " << static_cast<int>(action.lon)
         << ", lat: " << static_cast<int>(action.lat) << ", t: " << action.t
         << ")";
      return os;
    }

    DcpAction() {}
    DcpAction(const DcpLonAction& lon_, const DcpLatAction& lat_,
              const decimal_t& t_)
        : lon(lon_), lat(lat_), t(t_) {}
  };

  // 使用统一层时长构造 DCP-Tree。
  DcpTree(const int& tree_height, const decimal_t& layer_time);
  // 允许最后一层使用不同持续时间，以便与在线重规划周期更好对齐。
  DcpTree(const int& tree_height, const decimal_t& layer_time,
          const decimal_t& last_layer_time);
  ~DcpTree() = default;

  // ongoing_action_ 表示上一个规划周期已经开始执行的动作。
  // 新一轮树展开时，它会作为根动作被保留。
  void set_ongoing_action(const DcpAction& a) { ongoing_action_ = a; }

  // 返回当前整棵 DCP 树展开后的全部动作脚本。
  std::vector<std::vector<DcpAction>> action_script() const {
    return action_script_;
  }

  // 求整棵树当前动作脚本对应的总规划时域长度。
  decimal_t planning_horizon() const;

  int tree_height() const { return tree_height_; }

  decimal_t sim_time_per_layer() const { return layer_time_; }

  // 根据当前 ongoing_action_ 重新生成动作脚本。
  ErrorType UpdateScript();

  static std::string RetLonActionName(const DcpLonAction a) {
    std::string a_str;
    switch (a) {
      case DcpLonAction::kMaintain: {
        a_str = std::string("M");
        break;
      }
      case DcpLonAction::kAccelerate: {
        a_str = std::string("A");
        break;
      }
      case DcpLonAction::kDecelerate: {
        a_str = std::string("D");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  static std::string RetLatActionName(const DcpLatAction a) {
    std::string a_str;
    switch (a) {
      case DcpLatAction::kLaneKeeping: {
        a_str = std::string("K");
        break;
      }
      case DcpLatAction::kLaneChangeLeft: {
        a_str = std::string("L");
        break;
      }
      case DcpLatAction::kLaneChangeRight: {
        a_str = std::string("R");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

 private:
  // 按“单个规划周期最多一次动作变化”的规则生成所有候选策略。
  ErrorType GenerateActionScript();

  // 给现有动作前缀再追加 n 个相同动作。
  std::vector<DcpAction> AppendActionSequence(
      const std::vector<DcpAction>& seq_in, const DcpAction& a,
      const int& n) const;

  // 动作层数。
  int tree_height_ = 5;
  // 普通层持续时间。
  decimal_t layer_time_ = 1.0;
  // 最后一层持续时间，可单独设置用于时间对齐。
  decimal_t last_layer_time_ = 1.0;
  // 当前已在执行的动作。
  DcpAction ongoing_action_;
  // 全部根到叶动作序列。
  std::vector<std::vector<DcpAction>> action_script_;
};
}  // namespace planning

#endif  //  _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
