#include "dual_arm/small_qp.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <array>
#include <cmath>

namespace dual_arm::small_qp
{
  namespace
  {
    constexpr int kMaxK = kMaxVars + 1; // E 的行数上限 n + 1
    using EMatrix = Eigen::Matrix<double, kMaxK, kMaxRows>;
    using KVector = Eigen::Matrix<double, kMaxK, 1>;
    using SubMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxK, kMaxK>;
    using SubVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxK, 1>;
  } // namespace

  Result solveDiagonalQp(const RowMatrix &A, const RowVector &b, int m, int n, const VarVector &weight,
                         const VarVector &target, VarVector &x, Eigen::Matrix<bool, kMaxRows, 1> *active)
  {
    Result result;
    x = target;
    if (active)
      active->setConstant(false);
    if (m <= 0)
    {
      result.feasible = true;
      return result;
    }
    const int k = n + 1;

    // E = [Gᵀ; hᵀ]，G = −A W^{−1/2}，h = A t − b
    VarVector inv_sqrt_w = VarVector::Zero();
    for (int i = 0; i < n; ++i)
      inv_sqrt_w[i] = 1.0 / std::sqrt(weight[i]);
    EMatrix E = EMatrix::Zero();
    for (int j = 0; j < m; ++j)
    {
      double h = -b[j];
      for (int i = 0; i < n; ++i)
      {
        E(i, j) = -A(j, i) * inv_sqrt_w[i];
        h += A(j, i) * target[i];
      }
      E(n, j) = h;
    }
    KVector f = KVector::Zero();
    f[n] = 1.0;
    const double scale = std::max(1.0, E.topLeftCorner(k, m).cwiseAbs().maxCoeff());
    const double tol = 1e-12 * scale;

    // Lawson–Hanson NNLS：min ‖E u − f‖, u ≥ 0
    RowVector u = RowVector::Zero();
    Eigen::Matrix<bool, kMaxRows, 1> passive;
    passive.setConstant(false);
    std::array<int, kMaxRows> index{};
    SubMatrix Ep;
    SubVector sp;
    RowVector s = RowVector::Zero();

    for (int outer = 0; outer < 3 * kMaxRows; ++outer)
    {
      result.iterations = outer + 1;
      // E、u 在有效块之外全为 0，直接用定长乘积（避免把 k 维表达式赋给 5 维定长向量）
      const KVector residual = f - E * u;
      int enter = -1;
      double best = tol;
      for (int j = 0; j < m; ++j)
      {
        if (passive[j])
          continue;
        const double gradient = E.col(j).dot(residual);
        if (gradient > best)
        {
          best = gradient;
          enter = j;
        }
      }
      if (enter < 0)
        break;
      passive[enter] = true;

      for (int inner = 0; inner < 3 * kMaxRows; ++inner)
      {
        int p = 0;
        for (int j = 0; j < m; ++j)
          if (passive[j])
            index[p++] = j;
        if (p > k)
        {
          // 被动集列数超过行数说明列已线性相关（数值退化）：撤回新加入的列并结束。
          passive[enter] = false;
          outer = 3 * kMaxRows;
          break;
        }
        Ep.resize(k, p);
        for (int c = 0; c < p; ++c)
          Ep.col(c) = E.col(index[c]).head(k);
        sp = Eigen::ColPivHouseholderQR<SubMatrix>(Ep).solve(f.head(k));
        s.setZero();
        bool all_positive = true;
        for (int c = 0; c < p; ++c)
        {
          s[index[c]] = sp[c];
          all_positive &= sp[c] > tol;
        }
        if (all_positive)
        {
          u = s;
          break;
        }
        double alpha = 1.0;
        for (int c = 0; c < p; ++c)
        {
          const int j = index[c];
          if (s[j] <= tol)
            alpha = std::min(alpha, u[j] / std::max(u[j] - s[j], 1e-300));
        }
        for (int c = 0; c < p; ++c)
        {
          const int j = index[c];
          u[j] += alpha * (s[j] - u[j]);
          if (u[j] <= tol)
          {
            u[j] = 0.0;
            passive[j] = false;
          }
        }
      }
    }

    const KVector r = E * u - f;
    if (r.norm() < 1e-9 || r[n] > -1e-12)
      return result; // 不可行（Lawson–Hanson：‖r‖ = 0）
    result.feasible = true;
    for (int i = 0; i < n; ++i)
      x[i] = target[i] - inv_sqrt_w[i] * r[i] / r[n];
    for (int j = 0; j < m; ++j)
    {
      const bool on = u[j] > 0.0;
      result.active += on ? 1 : 0;
      if (active)
        (*active)[j] = on;
    }
    return result;
  }

} // namespace dual_arm::small_qp
