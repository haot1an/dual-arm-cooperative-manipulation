#include "dual_arm/closed_chain_ik.hpp"

#include "dual_arm/math_utils.hpp"

#include <Eigen/Cholesky>

namespace dual_arm
{

  bool solveClosedChainIk(RobotModel &model, const Pose &object, std::array<Vector7d, kNumArms> &q,
                          int max_iterations)
  {
    const SceneSpec &scene = model.scene();
    DualArmState state;
    for (Arm a : kArms)
    {
      state.arm(a).q = q[armIndex(a)];
      state.arm(a).dq.setZero();
    }
    bool converged = false;
    for (int iteration = 0; iteration < max_iterations && !converged; ++iteration)
    {
      model.update(state);
      converged = true;
      for (Arm a : kArms)
      {
        const Pose target = object * scene.graspOf(a).site_in_body;
        const Vector6d e = poseError(target, model.eePose(a));
        if (e.head<3>().norm() < 1e-7 && e.tail<3>().norm() < 1e-6)
          continue;
        converged = false;
        const Matrix6x7d J = model.jacobian(a);
        Vector7d dq = J.transpose() * (J * J.transpose() + 1e-6 * Matrix6d::Identity()).ldlt().solve(e);
        const double step = dq.norm();
        if (step > 0.2)
          dq *= 0.2 / step;
        Vector7d &qa = state.arm(a).q;
        qa = (qa + dq).cwiseMax(model.jointLowerLimit(a)).cwiseMin(model.jointUpperLimit(a));
      }
    }
    for (Arm a : kArms)
      q[armIndex(a)] = state.arm(a).q;
    return converged;
  }

  Eigen::Matrix<double, 7, 6> jointRateFromObjectTwist(const RobotModel &model, Arm arm, const Vector3d &object_origin)
  {
    const Matrix6x7d J = model.jacobian(arm);
    Matrix6d Gt = Matrix6d::Identity();
    Gt.topRightCorner<3, 3>() = -skew(model.eePose(arm).p - object_origin);
    return J.transpose() * (J * J.transpose() + 1e-6 * Matrix6d::Identity()).ldlt().solve(Gt);
  }

} // namespace dual_arm
