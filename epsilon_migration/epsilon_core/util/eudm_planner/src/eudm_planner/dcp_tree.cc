/**
 * @file dcp_tree.cc
 * @author GW
 * @brief DCP-Tree 实现：生成 EUDM 的语义动作脚本
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {

/**
 * @brief 使用统一层时长构造 DCP-Tree
 * @param tree_height 树高
 * @param layer_time 普通层持续时间
 *
 * **功能**：
 * 初始化树深与层时长，并立即生成动作脚本。
 */
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  // 若未单独指定最后一层时长，则默认与普通层一致。
  last_layer_time_ = layer_time_;
  GenerateActionScript();
}

/**
 * @brief 使用单独末层时长构造 DCP-Tree
 * @param tree_height 树高
 * @param layer_time 普通层持续时间
 * @param last_layer_time 最后一层持续时间
 *
 * **功能**：
 * 允许最后一层使用与普通层不同的时长，以适配在线重规划的时间对齐需求。
 */
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time,
                 const decimal_t& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  GenerateActionScript();
}

/**
 * @brief 重新生成动作脚本
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **功能**：
 * 在 `ongoing_action_` 更新后，重新展开整棵 DCP-Tree。
 */
ErrorType DcpTree::UpdateScript() { return GenerateActionScript(); }

/**
 * @brief 在现有动作序列后追加若干个相同动作
 * @param seq_in 输入动作序列前缀
 * @param a 需要追加的动作
 * @param n 追加次数
 * @return 返回扩展后的动作序列
 *
 * **用途**：
 * 用于快速构造“从第 h 层开始切换，并保持到末尾”的策略脚本。
 */
std::vector<DcpTree::DcpAction> DcpTree::AppendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const {
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}

/**
 * @brief 生成整棵树的动作脚本
 * @return 返回错误类型，kSuccess 表示成功
 *
 * **核心规则**（对应论文第 4.2 节）：
 * 在一个规划周期内，每条策略序列最多只允许一次动作变化。
 *
 * **生成逻辑**：
 * 1. 固定一类纵向语义动作（Maintain / Accelerate / Decelerate）
 * 2. 保留当前 `ongoing_action_` 的横向动作作为前缀
 * 3. 枚举“在第 h 层切换到另一横向动作，并保持到末尾”的所有可能
 * 4. 额外保留“全程不切换”的动作序列
 * 5. 最后一层单独覆盖为 `last_layer_time_`
 *
 * **复杂度意义**：
 * - 传统穷举会随树高指数增长
 * - DCP-Tree 通过先验约束，把复杂度压缩到与树高近似线性相关
 */
ErrorType DcpTree::GenerateActionScript() {
  // 这里实现论文中的关键剪枝:
  // 在一个规划周期内，每条策略最多只允许发生一次动作变化。
  //
  // 因此不会枚举所有可能的动作切换组合，而是只保留少量高价值语义脚本。
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) {
    // 先固定一类纵向风格，再在横向上生成“保持一段后切换”的脚本。
    ongoing_action_seq.clear();
    ongoing_action_seq.push_back(
        DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    for (int h = 1; h < tree_height_; ++h) {
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT);
           lat++) {
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          // 从第 h 层开始切到另一种横向动作，并保持到规划结束。
          auto actions = AppendActionSequence(
              ongoing_action_seq,
              DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_),
              tree_height_ - h);
          action_script_.push_back(actions);
        }
      }
      // 若不切换，则继续沿当前 ongoing action 延长一层。
      ongoing_action_seq.push_back(
          DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }
    // “全程不切换”的序列也要保留。
    action_script_.push_back(ongoing_action_seq);
  }
  // 单独覆盖最后一层时长，便于与实际规划周期更好对齐。
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }
  return kSuccess;
}

/**
 * @brief 计算当前动作脚本的规划时域
 * @return 返回总规划时长
 *
 * **功能**：
 * 将任意一条根到叶动作序列的各层持续时间累加，得到总规划时域。
 *
 * **说明**：
 * 所有脚本层数一致，因此取 `action_script_[0]` 即可。
 */
decimal_t DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  // 任取一条动作序列，把每层持续时间相加即可得到总规划时域。
  decimal_t planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace planning
