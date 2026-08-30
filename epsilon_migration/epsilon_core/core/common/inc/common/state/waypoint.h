/**
 * @file waypoint.h
 * @author GW
 * @brief 路点结构头文件
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义轨迹和路径模块共享的路点结构。
 */
#ifndef _COMMON_INC_COMMON_STATE_WAYPOINT_H__
#define _COMMON_INC_COMMON_STATE_WAYPOINT_H__

#include "common/basics/basics.h"
namespace common {

template <int N_DIM>
struct Waypoint {
  Vecf<N_DIM> pos;
  Vecf<N_DIM> vel;
  Vecf<N_DIM> acc;
  Vecf<N_DIM> jrk;
  decimal_t t{0.0};
  bool fix_pos = false;
  bool fix_vel = false;
  bool fix_acc = false;
  bool fix_jrk = false;
  bool stamped = false;
};

typedef Waypoint<2> Waypoint2D;
typedef Waypoint<3> Waypoint3D;
}  // namespace common

#endif
