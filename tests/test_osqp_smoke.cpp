// 仅在 -DDUAL_ARM_WITH_OSQP=ON 时编译：确认 OsqpEigen 能链接并求解一个小 QP，
// 为以后的力矩级 QP 做准备。
//   min ½ xᵀ H x + gᵀ x,  s.t. l ≤ A x ≤ u
#include <OsqpEigen/OsqpEigen.h>
#include <gtest/gtest.h>

TEST(OsqpSmoke, SolvesBoxConstrainedQp) {
  Eigen::SparseMatrix<double> H(2, 2), A(2, 2);
  H.insert(0, 0) = 2.0;
  H.insert(1, 1) = 2.0;
  A.insert(0, 0) = 1.0;
  A.insert(1, 1) = 1.0;
  Eigen::Vector2d g(-2.0, -4.0), lo(-10.0, -10.0), up(0.5, 10.0);

  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setAbsoluteTolerance(1e-8);
  solver.settings()->setRelativeTolerance(1e-8);
  solver.data()->setNumberOfVariables(2);
  solver.data()->setNumberOfConstraints(2);
  ASSERT_TRUE(solver.data()->setHessianMatrix(H));
  ASSERT_TRUE(solver.data()->setGradient(g));
  ASSERT_TRUE(solver.data()->setLinearConstraintsMatrix(A));
  ASSERT_TRUE(solver.data()->setLowerBound(lo));
  ASSERT_TRUE(solver.data()->setUpperBound(up));
  ASSERT_TRUE(solver.initSolver());
  ASSERT_EQ(solver.solveProblem(), OsqpEigen::ErrorExitFlag::NoError);
  const Eigen::VectorXd x = solver.getSolution();
  EXPECT_NEAR(x[0], 0.5, 1e-4);  // 无约束最优 (1, 2)，x0 被上界 0.5 截断
  EXPECT_NEAR(x[1], 2.0, 1e-4);
}
