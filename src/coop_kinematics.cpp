// 公式与坐标约定见 include/dual_arm/coop_kinematics.hpp
// 和 docs/cooperative_control.md。
#include "dual_arm/coop_kinematics.hpp"

#include "dual_arm/math_utils.hpp"

#include <cmath>
#include <stdexcept>
namespace dual_arm::coop
{
  
Twist screwMotionBasisAtPoint(
    const Vector3d& screw_axis_world,
    const Vector3d& axis_point_world,
    const Vector3d& reference_point_world,
    double lead)
{
  const double axis_norm =
      screw_axis_world.norm();

  if (axis_norm < 1e-12)
  {
    throw std::invalid_argument(
        "screwMotionBasisAtPoint: screw axis must be non-zero");
  }

  const Vector3d axis =
      screw_axis_world / axis_norm;

  const double pitch =
      lead / (2.0 * M_PI);

  const Vector3d radius =
      reference_point_world -
      axis_point_world;

  Twist basis =
      Twist::Zero();

  basis.head<3>() =
      pitch * axis +
      axis.cross(radius);

  basis.tail<3>() =
      axis;

  return basis;
}

Matrix6d screwConstraintProjector(
    const Twist& screw_basis)
{
  const double squared_norm =
      screw_basis.squaredNorm();

  if (squared_norm < 1e-12)
  {
    throw std::invalid_argument(
        "screwConstraintProjector: screw basis must be non-zero");
  }

  Matrix6d projector =
      Matrix6d::Identity();

  projector.noalias() -=
      screw_basis *
      screw_basis.transpose() /
      squared_norm;

  return projector;
}
  Matrix6d graspMatrixArm(const Vector3d &r_i)
  {
    Matrix6d G_i = Matrix6d::Zero();
    G_i.topLeftCorner<3, 3>().setIdentity();
    G_i.bottomLeftCorner<3, 3>() = -skew(r_i);
    G_i.bottomRightCorner<3, 3>().setIdentity();
    return G_i;
  }

  Matrix6x12d graspMatrix(const Vector3d &r_1, const Vector3d &r_2)
  {

    //  G = [G_1 G_2]
    Matrix6x12d G = Matrix6x12d::Zero();
    Matrix6d G_1 = graspMatrixArm(r_1);
    Matrix6d G_2 = graspMatrixArm(r_2);

    G.block<6, 6>(0, 0) = G_1;
    G.block<6, 6>(0, 6) = G_2;
    return G;
  }
Vector12d internalWrenchFromCoordinates(
    const Vector3d& r_1,
    const Vector3d& r_2,
    const Vector6d& h_r) {
  const Matrix6d G_1 =
      graspMatrixArm(r_1);

  const Matrix6d G_2 =
      graspMatrixArm(r_2);

  Vector12d h_internal;

  h_internal.head<6>() =
      -G_1.inverse() * h_r;

  h_internal.tail<6>() =
      G_2.inverse() * h_r;

  return h_internal;
}
  Matrix6x14d absoluteJacobian(const Matrix6x7d &J_1, const Matrix6x7d &J_2,
                               const Vector3d &r_1, const Vector3d &r_2)
  {
    Matrix6x7d J_1_object = J_1;
    Matrix6x7d J_2_object = J_2;
    J_1_object.topRows<3>().noalias() -=
        skew(r_1) * J_1.bottomRows<3>();

    J_2_object.topRows<3>().noalias() -=
        skew(r_2) * J_2.bottomRows<3>();

    Matrix6x14d J_a;
    J_a.leftCols<7>() = 0.5 * J_1_object;
    J_a.rightCols<7>() = 0.5 * J_2_object;
    return J_a;
  }

  Matrix6x14d relativeJacobian(const Matrix6x7d &J_1, const Matrix6x7d &J_2,
                               const Vector3d &r_1, const Vector3d &r_2)
  {

    //  J_r = [-G_1^{-T} J_1, G_2^{-T} J_2]
    Matrix6x7d J_1_object = J_1;
    Matrix6x7d J_2_object = J_2;

    J_1_object.topRows<3>().noalias() -=
        skew(r_1) * J_1.bottomRows<3>();

    J_2_object.topRows<3>().noalias() -=
        skew(r_2) * J_2.bottomRows<3>();

    Matrix6x14d J_r;
    J_r.leftCols<7>() = -J_1_object;
    J_r.rightCols<7>() = J_2_object;
    return J_r;
  }

  std::array<EeReference, kNumArms> eeReferencesFromObject(
      const ObjectReference &object_ref, const std::array<Pose, kNumArms> &grasp_in_object)
  {

    std::array<EeReference, kNumArms> ee_ref;
    const Vector3d omega_o = object_ref.twist.tail<3>();

    for (int i = 0; i < kNumArms; ++i)
    {
      ee_ref[i].pose = object_ref.pose * grasp_in_object[i];

      const Vector3d r_i =
          object_ref.pose.p - ee_ref[i].pose.p;

      const Matrix6d G_i = graspMatrixArm(r_i);

      ee_ref[i].twist.noalias() =
          G_i.transpose() * object_ref.twist;

      ee_ref[i].accel.noalias() =
          G_i.transpose() * object_ref.accel;

      ee_ref[i].accel.head<3>() -=
          omega_o.cross(omega_o.cross(r_i));
    }

    return ee_ref;
  }

  Matrix12x6d weightedPseudoInverse(
      const Matrix6x12d &G,
      const Matrix12d &W)
  {
    const Matrix12x6d W_inv_Gt =
        W.ldlt().solve(G.transpose());

    const Matrix6d A = G * W_inv_Gt;
    const Eigen::LDLT<Matrix6d> A_ldlt(A);

    return A_ldlt.solve(W_inv_Gt.transpose()).transpose();
  }

  Vector12d distributeObjectWrench(const Matrix6x12d &G, const Wrench &w_o,
                                   const Vector12d &h_int, const Matrix12d &W)
  {

    //  h = G_W^+ w_o + (I - G_W^+ G) h_int
    Matrix12x6d G_pinv = weightedPseudoInverse(G, W);
    Vector12d h_ext = G_pinv * w_o;
    Matrix12d N = Matrix12d::Identity() - G_pinv * G;
    Vector12d h = h_ext + N * h_int;

    return h;
  }

  Vector12d internalWrench(const Matrix6x12d &G, const Vector12d &h_meas)
  {

    //  h_int = (I - G^+ G) h_meas
    Matrix6d A = G * G.transpose();
    Matrix12x6d G_pinv = A.ldlt().solve(G).transpose();
    Vector12d h_int = h_meas - G_pinv * (G * h_meas);

    return h_int;
  }

  Vector6d internalWrenchCoordinates(const Vector3d &r_1, const Vector3d &r_2,
                                     const Vector12d &h)
  {

    // h_r = 0.5 * (G_2 h_2 - G_1 h_1)
    Matrix6d G_1 = graspMatrixArm(r_1);
    Matrix6d G_2 = graspMatrixArm(r_2);
    Vector6d h_r = 0.5 * (G_2 * h.tail<6>() - G_1 * h.head<6>());

    return h_r;
  }
  Vector12d internalWrenchForLogging(
      const DualArmState &state,
      const RobotModel &model)
  {
    const Vector3d p_1 = model.eePose(Arm::Left).p;
    const Vector3d p_2 = model.eePose(Arm::Right).p;
    const Vector3d p_o = state.object.pose.p;

    const Vector3d r_1 = p_o - p_1;
    const Vector3d r_2 = p_o - p_2;
    const Matrix6x12d G = graspMatrix(r_1, r_2);

    Vector12d h_meas;
    h_meas.head<6>() = state.arm(Arm::Left).ft_ee_world;
    h_meas.tail<6>() = state.arm(Arm::Right).ft_ee_world;

    return internalWrench(G, h_meas);
  }

} // namespace dual_arm::coop
