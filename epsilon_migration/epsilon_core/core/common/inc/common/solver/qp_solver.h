#ifndef _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__
#define _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__

#include "common/basics/basics.h"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

namespace common {

/**
 * @brief 将加权最小二乘样条问题改写为标准二次规划
 *
 * 该封装供样条拟合使用：`A x` 是由待求系数/控制点产生的观测值，`S` 为
 * 拟合权重，`W` 为系数正则项；随后交由 OOQP 处理边界、连续性等约束。
 */

class QuadraticProblem {
 public:
  /*!
   * 求解 (Ax-b)' S (Ax-b) + x' W x，其中 `C x = c` 是硬等式，
   * `d <= D x <= f` 是双边不等式，`l <= x <= u` 是变量边界。
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] C a (possibly null) matrix (m_cxn)
   * @param [in] c a vector (m_cx1)
   * @param [in] D a (possibly null) matrix (m_dxn)
   * @param [in] d a vector (m_dx1)
   * @param [in] f a vector (m_dx1)
   * @param [in] l a vector (m_dx1)
   * @param [in] u a vector (m_dx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x);
  /*!
   * 仅含等式约束的快捷接口；变量边界和不等式默认均为无穷宽。
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] C a (possibly null) matrix (m_cxn)
   * @param [in] c a vector (m_cx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c, Eigen::VectorXd& x);
};

}  // namespace common

#endif
