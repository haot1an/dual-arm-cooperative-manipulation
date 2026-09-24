#include "dual_arm/sparse_qp.hpp"

#include <Eigen/SparseCholesky>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dual_arm::sparse_qp
{

  Result solve(const Eigen::SparseMatrix<double> &P, const Eigen::VectorXd &q,
               const Eigen::SparseMatrix<double> &A, const Eigen::VectorXd &l, const Eigen::VectorXd &u,
               Eigen::VectorXd &x, const Settings &settings)
  {
    const Eigen::Index n = P.rows();
    const Eigen::Index m = A.rows();
    if (P.cols() != n || q.size() != n || A.cols() != n || l.size() != m || u.size() != m)
      throw std::invalid_argument("sparse_qp::solve: dimension mismatch");

    if (x.size() != n)
      x = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd z = (A * x).cwiseMax(l).cwiseMin(u);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(m);

    const double kEqualityScale = 1e3;
    double rho = settings.rho;
    Eigen::VectorXd rho_vec(m);
    auto updateRho = [&]()
    {
      for (Eigen::Index i = 0; i < m; ++i)
        rho_vec[i] = std::abs(u[i] - l[i]) < 1e-12 ? kEqualityScale * rho : rho;
    };
    updateRho();

    Eigen::SparseMatrix<double> identity(n, n);
    identity.setIdentity();
    const Eigen::SparseMatrix<double> At = A.transpose();
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
    auto factorize = [&]()
    {
      const Eigen::SparseMatrix<double> K = P + settings.sigma * identity + At * rho_vec.asDiagonal() * A;
      ldlt.compute(K);
      if (ldlt.info() != Eigen::Success)
        throw std::runtime_error("sparse_qp::solve: KKT factorization failed");
    };
    factorize();

    Result result;
    Eigen::VectorXd x_tilde(n), z_hat(m), rhs(n);
    for (int k = 1; k <= settings.max_iterations; ++k)
    {
      rhs = settings.sigma * x - q + At * (rho_vec.cwiseProduct(z) - y);
      x_tilde = ldlt.solve(rhs);
      z_hat = settings.alpha * (A * x_tilde) + (1.0 - settings.alpha) * z;
      x = settings.alpha * x_tilde + (1.0 - settings.alpha) * x;
      const Eigen::VectorXd z_new = (z_hat + y.cwiseQuotient(rho_vec)).cwiseMax(l).cwiseMin(u);
      y += rho_vec.cwiseProduct(z_hat - z_new);
      z = z_new;
      result.iterations = k;

      const bool check = k % settings.check_interval == 0 || k == settings.max_iterations;
      const bool adapt = k % settings.adapt_interval == 0;
      if (!check && !adapt)
        continue;
      const Eigen::VectorXd Ax = A * x;
      const Eigen::VectorXd Px = P * x;
      const Eigen::VectorXd Aty = At * y;
      result.primal_residual = m > 0 ? (Ax - z).cwiseAbs().maxCoeff() : 0.0;
      result.dual_residual = (Px + q + Aty).cwiseAbs().maxCoeff();
      const double primal_scale = m > 0 ? std::max(Ax.cwiseAbs().maxCoeff(), z.cwiseAbs().maxCoeff()) : 0.0;
      const double dual_scale = std::max({Px.cwiseAbs().maxCoeff(), Aty.cwiseAbs().maxCoeff(),
                                          q.cwiseAbs().maxCoeff()});
      if (result.primal_residual <= settings.eps_abs + settings.eps_rel * primal_scale &&
          result.dual_residual <= settings.eps_abs + settings.eps_rel * dual_scale)
      {
        result.converged = true;
        return result;
      }
      if (adapt && m > 0)
      {
        // OSQP 的 ρ 自适应：按归一化残差比缩放，比例变化大于 5 倍才重新分解
        const double ratio = std::sqrt((result.primal_residual / std::max(primal_scale, 1e-12)) /
                                       std::max(result.dual_residual / std::max(dual_scale, 1e-12), 1e-12));
        const double new_rho = std::clamp(rho * ratio, 1e-6, 1e6);
        if (new_rho > 5.0 * rho || new_rho < 0.2 * rho)
        {
          rho = new_rho;
          updateRho();
          factorize();
        }
      }
    }
    return result;
  }

} // namespace dual_arm::sparse_qp
