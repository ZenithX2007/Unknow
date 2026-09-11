#ifndef _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__
#define _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__

#include "common/spline/polynomial.h"
#include "common/state/frenet_state.h"

namespace common {

class FrenetPrimitive {
 public:
  FrenetPrimitive() {}
  /**
   * @brief 用最小 jerk 五次多项式连接两个 Frenet 状态
   *
   * 纵向始终以时间 t 参数化。高速模式下横向使用 d(t)，使横纵向可独立
   * 规划；低速或路径几何模式下横向使用 d(s)，避免低速时 d(t) 到 d(s)
   * 的链式法则换算病态。
   * @param is_lateral_independent 为 true 时启用 d(t) 横向模式
   */
  ErrorType Connect(const FrenetState& fs0, const FrenetState& fs1,
                    const decimal_t stamp, const decimal_t T,
                    bool is_lateral_independent);
  /**
   * @brief 以常加速度控制前向传播一个 Frenet 基元
   * @param u 纵向和横向加速度，横向固定采用 d(t) 表示
   */
  ErrorType Propagate(const FrenetState& fs0, const Vecf<2>& u,
                      const decimal_t stamp, const decimal_t T);

  /**
   * @brief Get the start point of the parameterization
   */
  decimal_t begin() const { return stamp_; }

  /**
   * @brief Get the end point of the parameterization
   */
  decimal_t end() const { return stamp_ + duration_; }

  /**
   * @brief Get the frenet state from the primitive
   * @note  when t is outside of the parameterization, extrapolation will be
   * applied
   */
  ErrorType GetFrenetState(const decimal_t t_global, FrenetState* fs) const;

  ErrorType GetFrenetStateSamples(const decimal_t step, const decimal_t offset,
                                  vec_E<FrenetState>* fs_vec) const;

  ErrorType GetJ(decimal_t* c_s, decimal_t* c_d) const;

  decimal_t lateral_T() const;
  decimal_t longitudial_T() const;

  FrenetState fs1() const;
  FrenetState fs0() const;

  Polynomial<5> poly_s() const { return poly_s_; }
  Polynomial<5> poly_d() const { return poly_d_; }

  void set_poly_s(const Polynomial<5>& poly) { poly_s_ = poly; }
  void set_poly_d(const Polynomial<5>& poly) { poly_d_ = poly; }
  /**
   * @brief Debug print
   */
  void print() const {
    printf("frenet primitive in duration [%lf, %lf].\n", begin(), end());
    poly_s_.print();
    poly_d_.print();
    fs0_.print();
    fs1_.print();
  }

  bool is_lateral_independent_ = false;

 private:
  Polynomial<5> poly_s_;
  Polynomial<5> poly_d_;
  decimal_t stamp_{0.0};
  decimal_t duration_{0.0};
  FrenetState fs0_;
  FrenetState fs1_;
  // d(s) 模式下纵向位移过小时，改用较长的虚拟弧长以避免多项式退化。
  decimal_t kSmallDistanceThreshold_ = 2.0;
};

}  // namespace common

#endif
