#ifndef _CORE_COMMON_INC_COMMON_LANE_LANE_H__
#define _CORE_COMMON_INC_COMMON_LANE_LANE_H__

/**
 * @file lane.h
 * @author GW
 * @brief 车道几何对象接口：基于样条曲线的车道中心线表示与查询
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 `common::Lane`，
 * 使用样条曲线表示车道中心线，并提供位置、切向、法向、曲率和弧长查询接口。
 */

#include "common/basics/config.h"
#include "common/spline/spline.h"
#include "common/state/state.h"

namespace common {

/**
 * @brief 样条化车道几何对象
 *
 * Lane 的自变量是沿中心线累计的弧长 s，而非普通多项式参数。
 * 因此可直接作为 Frenet 坐标系参考线：位置、切线、法线、朝向
 * 和曲率都可由同一个 s 查询得到。
 */
class Lane {
 public:
  typedef Spline<LaneDegree, LaneDim> SplineType;
  // typedef SplineGenerator<LaneDegree, LaneDim> SplineGeneratorType;

  Lane() {}
  Lane(const SplineType& position_spline)
      : position_spline_(position_spline), is_valid_(true) {}
  // 无有效样条时不能用于状态投影或前向仿真。
  bool IsValid() const { return is_valid_; }

  /**
   * @brief 设置车道中心线样条
   * @param position_spline 以弧长为参数的二维位置样条
   */
  void set_position_spline(const SplineType& position_spline) {
    if (position_spline.vec_domain().empty()) return;
    position_spline_ = position_spline;
    is_valid_ = true;
  }

  /**
   * @brief 按弧长计算曲率及其导数
   *
   * 曲率用于 Frenet 状态转换和横向动力学约束；曲率导数用于
   * 更高阶几何计算。仅在二维车道上有定义。
   */
  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature,
                                    decimal_t* curvature_derivative) const;

  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature) const;

  /**
   * @brief 按弧长获取位置样条的指定阶导数
   * @param d 导数阶数；d=0 时返回位置
   * @param derivative 输出的二维向量
   */
  ErrorType GetDerivativeByArcLength(const decimal_t arc_length, const int d,
                                     Vecf<LaneDim>* derivative) const;

  ErrorType GetPositionByArcLength(const decimal_t arc_length,
                                   Vecf<LaneDim>* derivative) const;

  ErrorType GetTangentVectorByArcLength(const decimal_t arc_length,
                                        Vecf<LaneDim>* tangent_vector) const;

  ErrorType GetNormalVectorByArcLength(const decimal_t arc_length,
                                       Vecf<LaneDim>* normal_vector) const;

  ErrorType GetOrientationByArcLength(const decimal_t arc_length,
                                      decimal_t* angle) const;

  // 将世界坐标点投影到中心线，返回最近投影点对应的弧长 s。
  ErrorType GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
                                      decimal_t* arc_length) const;

  // 已有粗略 s 时，使用 Newton 迭代精化最近投影，适合连续轨迹查询。
  ErrorType GetArcLengthByVecPositionWithInitialGuess(
      const Vecf<LaneDim>& vec_position, const decimal_t& initial_guess,
      decimal_t* arc_length) const;

  // ErrorType GetArcLengthByVecPositionUsingBinarySearch(
  //     const Vecf<LaneDim>& vec_position, decimal_t* arc_length) const;

  ErrorType CheckInputArcLength(const decimal_t arc_length) const;

  SplineType position_spline() const { return position_spline_; }

  decimal_t begin() const { return position_spline_.begin(); }

  decimal_t end() const { return position_spline_.end(); }

  void print() const { position_spline_.print(); }

 private:
  SplineType position_spline_;
  bool is_valid_ = false;
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_LANE_LANE_H__
