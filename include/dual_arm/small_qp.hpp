#pragma once
/**
 * @file small_qp.hpp
 * @brief 小规模对角 Hessian QP 的精确求解（无动态内存分配）。
 *
 *   min_x  ½ Σ_i w_i (x_i − t_i)²     s.t.  A x ≤ b     （x ∈ Rⁿ, n ≤ kMaxVars；m ≤ kMaxRows 行）
 *
 * 做法：令 y = W^{1/2}(x − t)，问题化为最小距离问题（LDP）min ‖y‖ s.t. G y ≥ h，
 * G = −A W^{−1/2}，h = A t − b。按 Lawson & Hanson（Solving Least Squares Problems, 1974, 第 23 章）
 * 用 NNLS 求解：min ‖E u − f‖, u ≥ 0，E = [Gᵀ; hᵀ]，f = e_{n+1}；残差 r = E u − f，
 * ‖r‖ ≈ 0 ⇔ 不可行，否则 y = −r_{1:n} / r_{n+1}。NNLS 为有限步的有效集法，结果精确（至舍入误差）。
 * u_j > 0 的行即为起作用约束。
 */
#include <Eigen/Core>

namespace dual_arm::small_qp
{

  constexpr int kMaxVars = 4;
  constexpr int kMaxRows = 24;

  using RowMatrix = Eigen::Matrix<double, kMaxRows, kMaxVars>;
  using RowVector = Eigen::Matrix<double, kMaxRows, 1>;
  using VarVector = Eigen::Matrix<double, kMaxVars, 1>;

  struct Result
  {
    bool feasible = false;
    int active = 0;          ///< 起作用约束数
    int iterations = 0;      ///< NNLS 外层迭代次数
  };

  /**
   * @param A, b    前 m 行、前 n 列有效
   * @param weight  w_i > 0
   * @param target  t
   * @param x       输出：最优解（不可行时为 target）
   * @param active  可选输出：第 j 行是否起作用
   */
  Result solveDiagonalQp(const RowMatrix &A, const RowVector &b, int m, int n, const VarVector &weight,
                         const VarVector &target, VarVector &x,
                         Eigen::Matrix<bool, kMaxRows, 1> *active = nullptr);

} // namespace dual_arm::small_qp
