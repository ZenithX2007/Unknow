/**
 * @file ssc_planner.h
 * @author GW
 * @brief SSC 运动规划器接口：基于时空语义走廊与 Bézier/QP 优化的轨迹生成器
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件声明 `SscPlanner` 及其核心输入/输出缓存。
 * 它是 EPSILON 中承接 EUDM 行为层、向下生成可执行轨迹的运动规划器。
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_

#include <memory>
#include <set>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/primitive/frenet_primitive.h"
#include "common/spline/spline_generator.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_bezier_traj.h"
#include "common/trajectory/frenet_primitive_traj.h"
#include "ssc_config.pb.h"
#include "ssc_planner/map_interface.h"
#include "ssc_planner/ssc_map.h"

namespace planning {

/**
 * @brief SSC 运动规划器
 *
 * **核心职责**：
 * 1. 从 map interface 读取语义场景输入
 * 2. 将所有输入批量转换到 Frenet 坐标系
 * 3. 构建 SSC 地图与时空语义走廊
 * 4. 对每个行为候选执行 Bézier/QP 优化
 * 5. 选择最终执行轨迹
 *
 * **对应文档**：
 * - `paper/SSC.md` 第 5 节：SSC 生成
 * - `paper/SSC.md` 第 6 节：轨迹优化
 */
class SscPlanner : public Planner {
 public:
  // 这个类是 SSC 运动规划器本体。
  //
  // 对初学者来说，可以先把它理解成:
  // 1. 从 map_interface 读取当前语义场景
  // 2. 把输入全部转到 Frenet 坐标系
  // 3. 调用 SscMap 构建时空占据图和 corridor
  // 4. 对每个行为候选做 Bezier/QP 优化
  // 5. 选出最终执行轨迹
  using ObstacleMapType = uint8_t;
  using SscMapDataType = uint8_t;

  using Lane = common::Lane;
  using State = common::State;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using FrenetState = common::FrenetState;
  using FrenetTrajectory = common::FrenetTrajectory;
  using FrenetPrimitive = common::FrenetPrimitive;
  using FrenetBezierTrajectory = common::FrenetBezierTrajectory;
  using FrenetPrimitiveTrajectory = common::FrenetPrimitiveTrajectory;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  typedef common::BezierSpline<5, 2> BezierSpline;

  SscPlanner() = default;

  // 将外部语义地图接口挂到规划器上。
  // 之后 RunOnce() 所需的所有输入都从这里读取。
  ErrorType set_map_interface(SscPlannerMapItf* map_itf);

  // 强制设置本轮规划的起始状态。
  // 若不显式设置，则默认使用 map_interface 提供的当前 ego 状态。
  ErrorType set_initial_state(const State& state);
  // 返回 SSC 的时空地图后端。
  // 主要用于可视化、调试和查看 corridor / occupancy map 结果。
  SscMap* p_ssc_map() const { return p_ssc_map_; }

  // 返回当前 ego 的 Frenet 状态，常用于查看起点约束。
  FrenetState ego_frenet_state() const { return ego_frenet_state_; }

  // 返回当前 ego 的全局车辆对象。
  Vehicle ego_vehicle() const { return ego_vehicle_; }

  // 返回上游行为层给出的 ego 候选前向轨迹（全局坐标系）。
  vec_E<vec_E<Vehicle>> forward_trajs() const { return forward_trajs_; }

  // 返回每个成功行为分支对应的 QP / Bezier 结果。
  vec_E<BezierSpline> qp_trajs() const { return qp_trajs_; }

  // 返回本轮规划时间原点，一般等于规划起点时间戳。
  decimal_t time_origin() const { return time_origin_; }

  // 返回当前主要行为候选对应的周围车 Frenet 预测。
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs()
      const {
    return sur_vehicle_trajs_fs_;
  }

  // 返回“每个行为候选下，每辆周车的 Frenet 预测轨迹”全集。
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
  surround_forward_trajs_fs() const {
    return surround_forward_trajs_fs_;
  };

  // 返回 ego 候选参考轨迹的 Frenet 形式。
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs() const {
    return forward_trajs_fs_;
  }

  // 返回 ego 在 Frenet 空间中的车辆轮廓顶点。
  vec_E<Vec2f> ego_vehicle_contour_fs() const {
    return fs_ego_vehicle_.vertices;
  }

  // 返回 ego 的 Frenet 车辆包络。
  common::FsVehicle fs_ego_vehicle() const { return fs_ego_vehicle_; }

  // BezierSpline bezier_spline() const { return bezier_spline_; }

  std::unique_ptr<FrenetTrajectory> trajectory() const {
    // 统一对外暴露“最终轨迹”接口。
    //
    // 高速模式:
    //   返回 FrenetBezierTrajectory
    // 低速模式:
    //   返回 FrenetPrimitiveTrajectory
    //
    // 这样 server/可视化层不需要关心底层到底用了哪一种轨迹表达。
    if (!is_lateral_independent_) {
      return std::unique_ptr<FrenetPrimitiveTrajectory>(
          new FrenetPrimitiveTrajectory(low_spd_alternative_traj_));
    }
    return std::unique_ptr<FrenetBezierTrajectory>(
        new FrenetBezierTrajectory(trajectory_));
  }

  // 返回当前参考线对应的 Frenet 变换器。
  common::StateTransformer state_transformer() const { return stf_; }

  // 返回本轮规划总耗时。
  decimal_t time_cost() const { return time_cost_; }

  // 返回规划起点在 Frenet 中的状态。
  common::FrenetState initial_frenet_state() const {
    return initial_frenet_state_;
  }

  // 返回规划器名称。
  std::string Name() override;

  // 使用配置文件初始化规划器与 SSC map 配置。
  ErrorType Init(const std::string config_path) override;

  // 执行一次完整的运动规划周期。
  ErrorType RunOnce() override;

 private:
  // 读取 protobuf 配置文件。
  ErrorType ReadConfig(const std::string config_path);

  // 检查生成的 corridor 在时间上是否连续、是否满足最基本的几何合理性。
  ErrorType CorridorFeasibilityCheck(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes);
  // 将 ego、障碍物、参考轨迹、周围车预测批量转到 Frenet。
  ErrorType StateTransformForInputData();

  // 在已有 corridor 上做 Bezier/QP 优化，生成每个行为候选的平滑轨迹。
  ErrorType RunQpOptimization();

  // 对输出轨迹做基础合法性检查，例如曲率是否过大。
  ErrorType ValidateTrajectory(const FrenetTrajectory& traj);
  ErrorType StateTransformUsingOpenMp(const vec_E<State>& global_state_vec,
                                      const vec_E<Vec2f>& global_point_vec,
                                      vec_E<FrenetState>* frenet_state_vec,
                                      vec_E<Vec2f>* fs_point_vec) const;

  // 单线程版本的 Frenet 批量变换，实现更直接，也更适合调试。
  ErrorType StateTransformSingleThread(const vec_E<State>& global_state_vec,
                                       const vec_E<Vec2f>& global_point_vec,
                                       vec_E<FrenetState>* frenet_state_vec,
                                       vec_E<Vec2f>* fs_point_vec) const;

  // 在所有可行候选里，挑出与当前行为最匹配的一条最终轨迹。
  ErrorType UpdateTrajectoryWithCurrentBehavior();

  // 当前 ego 车辆的全局状态与几何信息。
  Vehicle ego_vehicle_;
  // 当前离散行为标签，例如 keep lane / lane change。
  LateralBehavior ego_behavior_;
  // ego 在参考车道 Frenet 坐标系下的状态。
  FrenetState ego_frenet_state_;
  // 当前局部参考车道，所有 Frenet 变换都依赖它。
  Lane nav_lane_local_;
  // 当前规划轮次对应的时间原点，一般取规划起始时间。
  decimal_t time_origin_{0.0};

  // 当前轮优化使用的初始状态。
  State initial_state_;
  // 标记本轮是否由外部显式指定了初始状态。
  // 若为 false，则 RunOnce() 默认取 ego 当前真实状态作为起点。
  bool has_initial_state_ = false;

  // initial_state_ 对应的 Frenet 形式。
  common::FrenetState initial_frenet_state_;

  // ==================== 上游输入缓存区 ====================
  // 这些成员保存的是从 map_interface 拿到的“原始输入”。
  //
  // 二维障碍图与静态障碍格点集合。
  GridMap2D grid_map_;
  std::set<std::array<decimal_t, 2>> obstacle_grids_;
  // 上游行为层提供的 ego 候选前向轨迹及其行为标签。
  vec_E<vec_E<Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  // 上游提供的周围车预测，用于构建动态障碍。
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;

  // ==================== Frenet 中间结果区 ====================
  // 静态障碍在 Frenet 平面中的离散点。
  vec_E<Vec2f> obstacle_grids_fs_;

  // 下面这些变量是“优化阶段真正使用的 Frenet 世界”。
  //
  // 可以理解成把上面的全局量，整理成更适合 SSC/QP 的中间结果。
  // Initial solution for optimization
  // ego 自身在 Frenet 中的车辆包络。
  common::FsVehicle fs_ego_vehicle_;
  // ego 候选参考轨迹的 Frenet 形式。
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs_;
  // 当前选中行为对应的周围车 Frenet 预测。
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs_;
  // 所有行为候选下的周围车 Frenet 预测。
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
      surround_forward_trajs_fs_;

  // ==================== 优化输出缓存区 ====================
  // 每个行为候选对应一条 Bezier/QP 结果。
  vec_E<BezierSpline> qp_trajs_;
  // 低速下的 primitive 结果。
  vec_E<FrenetPrimitive> primitive_trajs_;
  // 真正通过 corridor 检查并优化成功的行为集合。
  std::vector<LateralBehavior> valid_behaviors_;
  // 每个行为对应的时空语义走廊。
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> corridors_;
  // 每个行为对应的参考 Frenet 轨迹点。
  vec_E<vec_E<common::FrenetState>> ref_states_list_;

  // ==================== 最终输出区 ====================
  // 是否采用“纵横向近似解耦”的高速规划模式。
  bool is_lateral_independent_ = true;
  // 高速模式下最终输出的 Bezier 轨迹。
  FrenetBezierTrajectory trajectory_;
  // 低速模式下最终输出的 primitive 轨迹。
  FrenetPrimitiveTrajectory low_spd_alternative_traj_;
  // 最终选中的 corridor 与参考轨迹。
  vec_E<common::SpatioTemporalSemanticCubeNd<2>> final_corridor_;
  vec_E<common::FrenetState> final_ref_states_;

  // Frenet 变换器。
  common::StateTransformer stf_;
  // ==================== 系统依赖与统计区 ====================
  // 外部地图接口与内部 SSC map。
  SscPlannerMapItf* map_itf_;
  bool map_valid_ = false;
  SscMap* p_ssc_map_;

  // 统计信息。
  // stamp_     : 当前规划轮次对应的时间戳
  // time_cost_ : RunOnce() 总耗时
  decimal_t stamp_ = 0.0;
  decimal_t time_cost_ = 0.0;

  // protobuf 配置。
  planning::ssc::Config cfg_;
};

}  // namespace planning

#endif
