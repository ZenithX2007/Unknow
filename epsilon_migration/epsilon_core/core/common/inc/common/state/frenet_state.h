/**
 * @file frenet_state.h
 * @author GW
 * @brief Frenet 状态结构头文件：相对参考车道的状态表示
 * @version 0.1
 * @date 2019
 * @copyright Copyright (c) 2019
 *
 * **文件概述**：
 * 本文件定义 Frenet 坐标系下的状态结构，是规划与预测模块的重要基础表示。
 */
#ifndef _COMMON_INC_COMMON_STATE_FRENET_STATE_H__
#define _COMMON_INC_COMMON_STATE_FRENET_STATE_H__

namespace common {
/**
 * @brief 参考车道 Frenet 坐标系下的车辆状态
 *
 * `vec_s` 保存纵向弧长及其时间导数，`vec_dt` 保存横向 d
 * 对时间 t 的导数，`vec_ds` 保存横向 d 对纵向 s 的导数。
 * 高速运动规划通常使用 d(t) 表示横向运动；描述路径几何时，
 * 则使用 d(s) 表示横向偏移。
 */
struct FrenetState {
  enum InitType { kInitWithDt, kInitWithDs };
  decimal_t time_stamp{0.0};
  Vecf<3> vec_s{Vecf<3>::Zero()};   // [s, ds/dt, d2s/dt2]
  Vecf<3> vec_dt{Vecf<3>::Zero()};  // [d, dd/dt, d2d/dt2]
  Vecf<3> vec_ds{Vecf<3>::Zero()};  // [d, dd/ds, d2d/ds2]
  bool is_ds_usable = true;         // 速度接近 0 时 d(s) 换算可能失效

  FrenetState() {}
  void Load(const Vecf<3>& s, const Vecf<3>& d, const InitType& type) {
    vec_s = s;
    if (type == kInitWithDt) {
      // 已知横向时间导数时，通过链式法则换算 d(s)。
      // 当 ds/dt 接近 0 时除法不稳定，因此只保留 d(t) 表示。
      vec_dt = d;
      vec_ds[0] = vec_dt[0];
      if (fabs(vec_s[1]) > kEPS) {
        vec_ds[1] = vec_dt[1] / vec_s[1];
        vec_ds[2] = (vec_dt[2] - vec_ds[1] * vec_s[2]) / (vec_s[1] * vec_s[1]);
        is_ds_usable = true;
      } else {
        vec_ds[1] = 0.0;
        vec_ds[2] = 0.0;
        is_ds_usable = false;
      }
    } else if (type == kInitWithDs) {
      // 已知 d(s) 时，用 ds/dt 将路径导数换回时间导数。
      vec_ds = d;
      vec_dt[0] = vec_ds[0];
      vec_dt[1] = vec_s[1] * vec_ds[1];
      vec_dt[2] = vec_ds[2] * vec_s[1] * vec_s[1] + vec_ds[1] * vec_s[2];
      is_ds_usable = true;
    } else {
      assert(false);
    }
  }

  FrenetState(const Vecf<3>& s, const Vecf<3>& dt, const Vecf<3>& ds) {
    vec_s = s;
    vec_dt = dt;
    vec_ds = ds;
  }

  void print() const {
    printf("frenet state stamp: %lf.\n", time_stamp);
    printf("-- vec_s: (%lf, %lf, %lf).\n", vec_s[0], vec_s[1], vec_s[2]);
    printf("-- vec_dt: (%lf, %lf, %lf).\n", vec_dt[0], vec_dt[1], vec_dt[2]);
    printf("-- vec_ds: (%lf, %lf, %lf).\n", vec_ds[0], vec_ds[1], vec_ds[2]);
    if (!is_ds_usable) printf("-- warning: ds not usable.\n");
  }
};

}  // namespace common

#endif
