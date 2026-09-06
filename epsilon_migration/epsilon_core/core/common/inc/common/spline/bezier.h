#ifndef _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__

#include <assert.h>

#include <vector>

namespace common {
// forward declaration
template <int N_DEG>
class BezierUtils;

/**
 * @brief 分段 Bézier 样条
 *
 * 每段对应 SSC 的一个时空立方体，`ctrl_pts_` 保存该段各维度的控制点。
 * 求值时先将真实时间归一化到 [0, 1]，再通过 Bernstein 基函数线性组合
 * 控制点。由于 Bézier 曲线及其导数都位于对应控制点的凸包内，规划器可以
 * 直接约束控制点及相邻差分，从而保证整段的位置和动力学边界。
 */
template <int N_DEG, int N_DIM>
class BezierSpline {
 public:
  BezierSpline() {}
  /**
   * @brief Set the vector domain
   * @param vec_domain: input vec domain
   */
  void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
    assert(vec_domain.size() > 1);
    vec_domain_ = vec_domain;
    ctrl_pts_.resize(vec_domain.size() - 1);
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      ctrl_pts_[j].setZero();
    }
  }

  void set_coeff(const int segment_idx, const int ctrl_pt_index,
                 const Vecf<N_DIM>& coeff) {
    ctrl_pts_[segment_idx].row(ctrl_pt_index) = coeff;
  }

  void set_ctrl_pts(const vec_E<Matf<N_DEG + 1, N_DIM>>& pts) {
    ctrl_pts_ = pts;
  }

  /**
   * @brief Get the number of segments of the spline
   * @param n number of the segments
   */
  int num_segments() const { return static_cast<int>(ctrl_pts_.size()); }

  /**
   * @brief Get the vec domain of the spline
   * @param n number of the segments
   */
  std::vector<decimal_t> vec_domain() const { return vec_domain_; }

  /**
   * @brief
   *
   * @return vec_E<Matf<N_DEG + 1, N_DIM>>
   */
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts() const { return ctrl_pts_; }

  /**
   * @brief Get the begin of the parameterization
   */
  decimal_t begin() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.front();
  }

  /**
   * @brief Get the end of the parameterization
   */
  decimal_t end() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.back();
  }

  /**
   * @brief Return evaluation of spline at given parameterization
   * @param s parameterization, s should be in the vector domain
   * @param d derivate to take
   * @note  out of domain s will be rounded to the end()
   */
  ErrorType evaluate(const decimal_t s, const int d, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx =
        std::min(std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0),
                 num_pts - 2);
    decimal_t h = s - vec_domain_[idx];
    if (s < vec_domain_[0]) {
      return kIllegalInput;
    } else {
      decimal_t duration = vec_domain_[idx + 1] - vec_domain_[idx];
      decimal_t normalized_s = h / duration;
      // printf("derivative %d, normalized s: %lf.\n", d, normalized_s);
      // 导数相对归一化时间求得；duration 的幂次将其换回真实时间尺度。
      Vecf<N_DEG + 1> basis =
          BezierUtils<N_DEG>::GetBezierBasis(d, normalized_s);
      Vecf<N_DIM> result =
          (pow(duration, 1 - d) * basis.transpose() * ctrl_pts_[idx])
              .transpose();
      *ret = result;
    }
    return kSuccess;
  }

  void print() const {
    printf("Bezier control points.\n");
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      printf("segment %d -->.\n", j);
      std::cout << ctrl_pts_[j] << std::endl;
    }
  }

 private:
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts_;
  std::vector<decimal_t> vec_domain_;
};

template <int N_DEG>
class BezierUtils {
 public:
  /**
   * @brief 计算未做时间缩放时的 Bézier 导数平方积分 Hessian
   *
   * SSC 优化使用三阶导数（jerk）平方积分作为平滑度代价；各段时长的
   * 缩放在调用端根据真实时间域补入。
   */
  static MatNf<N_DEG + 1> GetBezierHessianMat(int derivative_degree) {
    MatNf<N_DEG + 1> hessian;
    switch (N_DEG) {
      case 5: {
        if (derivative_degree == 3) {
          // jerk hessian
          hessian << 720.0, -1800.0, 1200.0, 0.0, 0.0, -120.0, -1800.0, 4800.0,
              -3600.0, 0.0, 600.0, 0.0, 1200.0, -3600.0, 3600.0, -1200.0, 0.0,
              0.0, 0.0, 0.0, -1200.0, 3600.0, -3600.0, 1200.0, 0.0, 600.0, 0.0,
              -3600.0, 4800.0, -1800.0, -120.0, 0.0, 0.0, 1200.0, -1800.0,
              720.0;
          break;
        } else {
          assert(false);
        }
        break;
      }
      default:
        assert(false);
    }
    return hessian;
  }

  /**
   * @brief 计算控制点 [c_0, c_1, ...] 对应的 Bernstein 基函数及其导数
   *
   * 参数 `t` 会被夹到 [0, 1]，因此端点附近的数值查询仍使用合法的
   * Bézier 基函数。
   */
  static Vecf<N_DEG + 1> GetBezierBasis(int derivative_degree, decimal_t t) {
    t = std::max(std::min(1.0, t), 0.0);
    Vecf<N_DEG + 1> basis;
    switch (N_DEG) {
      case 5:
        if (derivative_degree == 0) {
          basis << -pow(t - 1, 5), 5 * t * pow(t - 1, 4),
              -10 * pow(t, 2) * pow(t - 1, 3), 10 * pow(t, 3) * pow(t - 1, 2),
              -5 * pow(t, 4) * (t - 1), pow(t, 5);
        } else if (derivative_degree == 1) {
          basis << -5 * pow(t - 1, 4),
              20 * t * pow(t - 1, 3) + 5 * pow(t - 1, 4),
              -20 * t * pow(t - 1, 3) - 30 * pow(t, 2) * pow(t - 1, 2),
              10 * pow(t, 3) * (2 * t - 2) + 30 * pow(t, 2) * pow(t - 1, 2),
              -20 * pow(t, 3) * (t - 1) - 5 * pow(t, 4), 5 * pow(t, 4);
        } else if (derivative_degree == 2) {
          basis << -20 * pow(t - 1, 3),
              60 * t * pow(t - 1, 2) + 40 * pow(t - 1, 3),
              -120 * t * pow(t - 1, 2) - 20 * pow(t - 1, 3) -
                  30 * t * t * (2 * t - 2),
              60 * t * pow(t - 1, 2) + 60 * t * t * (2 * t - 2) +
                  20 * t * t * t,
              -60 * t * t * (t - 1) - 40 * t * t * t, 20 * t * t * t;
        } else if (derivative_degree == 3) {
          basis << -60 * pow(t - 1, 2),
              60 * t * (2 * t - 2) + 180 * pow(t - 1, 2),
              -180 * t * (2 * t - 2) - 180 * pow(t - 1, 2) - 60 * t * t,
              180 * t * (2 * t - 2) + 60 * pow(t - 1, 2) + 180 * t * t,
              -120 * t * (t - 1) - 180 * t * t, 60 * t * t;
        }
        break;
      default:
        printf("N_DEG %d, derivative_degree %d.\n", N_DEG, derivative_degree);
        assert(false);
        break;
    }
    return basis;
  }
};

}  // namespace common

#endif
