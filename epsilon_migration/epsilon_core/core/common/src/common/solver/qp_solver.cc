#include "common/solver/qp_solver.h"

#include <stdexcept>

#include "common/solver/ooqp_interface.h"

namespace common {

bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x) {
  // 丢弃与 x 无关的常数 b'Sb 后，加权最小二乘可写成 OOQP 所需的
  // 1/2 x'Qx + c'x。x 在这里通常是一整条样条的系数或 Bézier 控制点。
  int m = A.rows();
  int n = A.cols();
  x.setZero(n);
  assert(static_cast<int>(b.size()) == m);
  assert(static_cast<int>(S.rows()) == m);
  assert(static_cast<int>(W.rows()) == n);
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_temp;
  Q_temp = A.transpose() * S * A +
           (Eigen::SparseMatrix<double, Eigen::RowMajor>)W.toDenseMatrix()
               .sparseView();
  // Q_temp = A.transpose() * S * A;
  Eigen::VectorXd c_temp = -A.transpose() * S * b;
  return OoQpItf::solve(Q_temp, c_temp, C, c, D, d, f, l, u, x);
}

bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c, Eigen::VectorXd& x) {
  // 此重载不施加变量盒约束，显式传入无穷边界以复用完整求解流程。
  int nx = A.cols();
  Eigen::VectorXd u =
      std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(nx);
  Eigen::VectorXd l = (-u.array()).matrix();
  // null D matrix
  Eigen::SparseMatrix<double, Eigen::RowMajor> D;
  Eigen::VectorXd d, f;
  return solve(A, S, b, W, C, c, D, d, f, l, u, x);

}

}  // namespace common
