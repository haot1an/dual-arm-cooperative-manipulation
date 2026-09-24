#pragma once
/**
 * @file sparse_qp.hpp
 * @brief 稀疏凸 QP 的 ADMM 求解器（OSQP 同形式，供离线轨迹优化使用；会动态分配内存，不用于 1 kHz 回路）。
 *
 *   min_x  ½ xᵀ P x + qᵀ x     s.t.  l ≤ A x ≤ u
 *
 * 迭代（Stellato 等，OSQP，2020）：
 *   (P + σI + Aᵀ R A) x̃ = σ x − q + Aᵀ (R z − y)
 *   x ← α x̃ + (1 − α) x,   ẑ = α A x̃ + (1 − α) z
 *   z ← Π_[l,u](ẑ + R⁻¹ y),   y ← y + R (ẑ − z)
 * R = ρ I（等式行 ρ × 10³），按原始 / 对偶残差比自适应调整 ρ 并重新分解。
 */
#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace dual_arm::sparse_qp
{

  struct Settings
  {
    double rho = 0.1;
    double sigma = 1e-6;
    double alpha = 1.6;
    int max_iterations = 8000;
    double eps_abs = 1e-7;
    double eps_rel = 1e-6;
    int check_interval = 10;
    int adapt_interval = 100;
  };

  struct Result
  {
    bool converged = false;
    int iterations = 0;
    double primal_residual = 0.0; ///< ‖A x − z‖∞
    double dual_residual = 0.0;   ///< ‖P x + q + Aᵀ y‖∞
  };

  /// P 须为完整对称矩阵（不是只存上三角）。x 同时作为热启动输入（尺寸不对时从 0 开始）。
  Result solve(const Eigen::SparseMatrix<double> &P, const Eigen::VectorXd &q,
               const Eigen::SparseMatrix<double> &A, const Eigen::VectorXd &l, const Eigen::VectorXd &u,
               Eigen::VectorXd &x, const Settings &settings = Settings{});

} // namespace dual_arm::sparse_qp
