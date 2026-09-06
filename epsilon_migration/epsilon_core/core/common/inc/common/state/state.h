#ifndef _COMMON_INC_COMMON_STATE_STATE_H__
#define _COMMON_INC_COMMON_STATE_STATE_H__

/**
 * @file state.h
 * @author GW
 * @brief 通用车辆状态结构体：位置、姿态、曲率、速度与加速度定义
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义最常用的全局车辆状态结构 `common::State`，
 * 作为规划、控制、预测和仿真模块之间共享的核心状态表示。
 */

#include "common/basics/basics.h"

namespace common {

/**
 * @brief 全局笛卡尔坐标系下的车辆状态
 *
 * 车辆位置默认取后轴中心，姿态角沿车辆纵轴方向定义。
 * 该结构是仿真器、预测器、行为规划器和运动规划器之间共享的
 * 基础状态格式；进入车道相关算法时，再由 StateTransformer
 * 转换为 FrenetState。
 */
struct State {
  decimal_t time_stamp{0.0};              // 状态时间戳，单位 s
  Vecf<2> vec_position{Vecf<2>::Zero()};  // 后轴中心世界坐标，单位 m
  decimal_t angle{0.0};                   // 车身航向角，单位 rad
  decimal_t curvature{0.0};               // 后轴轨迹曲率，单位 1/m
  decimal_t velocity{0.0};                // 车身纵向速度，单位 m/s
  decimal_t acceleration{0.0};            // 车身纵向加速度，单位 m/s^2
  decimal_t steer{0.0};                   // 前轮转角，单位 rad
  void print() const {
    printf("State:\n");
    printf(" -- time_stamp: %lf.\n", time_stamp);
    printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
    printf(" -- angle: %lf.\n", angle);
    printf(" -- curvature: %lf.\n", curvature);
    printf(" -- velocity: %lf.\n", velocity);
    printf(" -- acceleration: %lf.\n", acceleration);
    printf(" -- steer: %lf.\n", steer);
  }

  Vec3f ToXYTheta() const {
    // 车道匹配只需要平面位置和航向，不需要速度等动态量。
    return Vec3f(vec_position(0), vec_position(1), angle);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_STATE_H__
