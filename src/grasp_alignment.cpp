#include "dual_arm/grasp_alignment.hpp"

#include "dual_arm/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace dual_arm
{
  namespace
  {
    constexpr double kOpenFinger = 0.04;

    Pose offsetAlongApproach(const Pose &grasp, double distance)
    {
      Pose out = grasp;
      out.p += grasp.R().col(2) * distance;
      return out;
    }
  } // namespace

  GraspAlignment::GraspAlignment(std::shared_ptr<RobotModel> model, const GraspAlignmentConfig &config)
      : model_(std::move(model)), config_(config)
  {
    if (!model_)
      throw std::invalid_argument("GraspAlignment: model is null");
  }

  const char *GraspAlignment::phaseName(Phase phase)
  {
    switch (phase)
    {
    case Phase::Approach: return "APPROACH";
    case Phase::Align: return "ALIGN";
    case Phase::Close: return "CLOSE";
    case Phase::Done: return "DONE";
    case Phase::Failed: return "FAILED";
    }
    return "?";
  }

  std::array<Vector7d, kNumArms> GraspAlignment::preGraspConfiguration(
      const std::array<Pose, kNumArms> &grasp_targets)
  {
    const SceneSpec &scene = model_->scene();
    DualArmState state;
    for (Arm a : kArms)
    {
      state.arm(a).q = scene.q_init[armIndex(a)];
      state.arm(a).dq.setZero();
    }
    std::array<Vector7d, kNumArms> result;
    for (Arm a : kArms)
    {
      const int i = armIndex(a);
      const Pose target = offsetAlongApproach(grasp_targets[i], -scene.grasp[i].approach_distance);
      const Vector7d lower = model_->jointLowerLimit(a);
      const Vector7d upper = model_->jointUpperLimit(a);
      Vector7d &q = state.arm(a).q;
      bool converged = false;
      Vector6d e = Vector6d::Zero();
      for (int iteration = 0; iteration < 500 && !converged; ++iteration)
      {
        model_->update(state);
        e = poseError(target, model_->eePose(a));
        if (e.head<3>().norm() < 1e-5 && e.tail<3>().norm() < 1e-4)
        {
          converged = true;
          break;
        }
        const Matrix6x7d J = model_->jacobian(a);
        const Matrix6d A = J * J.transpose() + 1e-4 * Matrix6d::Identity();
        // 零空间把解拉向抓取构型，避免预抓取时腕部翻转；其增益逐步衰减到 0，
        // 使最后的迭代是纯任务空间修正（阻尼伪逆的零空间投影并不精确）。
        const Vector7d dq_task = J.transpose() * A.ldlt().solve(e);
        const Matrix7d N = Matrix7d::Identity() - J.transpose() * A.ldlt().solve(J);
        const double posture_gain = 0.1 * std::max(0.0, 1.0 - iteration / 300.0);
        q += dq_task + posture_gain * N * (scene.q_init[i] - q);
        q = q.cwiseMax(lower).cwiseMin(upper);
      }
      if (!converged)
        throw std::runtime_error(std::string("GraspAlignment: pre-grasp IK did not converge for ") + armName(a) +
                                 " (residual " + std::to_string(1e3 * e.head<3>().norm()) + " mm / " +
                                 std::to_string(e.tail<3>().norm() * 180.0 / M_PI) + " deg)");
      result[i] = q;
    }
    return result;
  }

  void GraspAlignment::reset(const DualArmState &state)
  {
    start_time_ = state.t;
    phase_ = Phase::Approach;
    phase_start_time_ = state.t;
    condition_start_time_ = -1.0;
    finger_target_ = {kOpenFinger, kOpenFinger};
    position_error_ = 0.0;
    orientation_error_ = 0.0;
  }

  void GraspAlignment::enterPhase(Phase phase, double t)
  {
    phase_ = phase;
    phase_start_time_ = t;
    condition_start_time_ = -1.0;
  }

  bool GraspAlignment::conditionHeld(bool condition, double t, double hold_time)
  {
    if (!condition)
    {
      condition_start_time_ = -1.0;
      return false;
    }
    if (condition_start_time_ < 0.0)
      condition_start_time_ = t;
    return t - condition_start_time_ >= hold_time;
  }

  std::pair<Vector7d, Vector7d> GraspAlignment::compute(
      const DualArmState &state,
      const std::array<Pose, kNumArms> &grasp_targets,
      const std::array<std::array<double, 2>, kNumArms> &finger_normals)
  {
    model_->update(state);
    const SceneSpec &scene = model_->scene();
    const double t = state.t;

    // 接近段 min-jerk：s = 10τ³ − 15τ⁴ + 6τ⁵，ṡ 两端为 0。
    double remaining = 0.0;
    double approach_speed = 0.0;
    if (phase_ == Phase::Approach)
    {
      const double tau = std::clamp((t - phase_start_time_) / config_.approach_time, 0.0, 1.0);
      const double s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
      const double ds = 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / config_.approach_time;
      remaining = 1.0 - s;
      approach_speed = ds;
    }

    std::array<Vector7d, kNumArms> tau;
    double max_position_error = 0.0;
    double max_orientation_error = 0.0;
    double max_speed = 0.0;
    for (Arm a : kArms)
    {
      const int i = armIndex(a);
      const double d = scene.grasp[i].approach_distance;
      const Pose target = offsetAlongApproach(grasp_targets[i], -d * remaining);
      Twist reference_twist = Twist::Zero();
      reference_twist.head<3>() = grasp_targets[i].R().col(2) * (d * approach_speed);

      const Twist twist = model_->eeTwist(a);
      const Vector6d e = poseError(target, model_->eePose(a));
      const Wrench w = config_.stiffness.cwiseProduct(e) + config_.damping.cwiseProduct(reference_twist - twist);
      const Matrix6x7d J = model_->jacobian(a);
      const Matrix6d A = J * J.transpose() + 1e-6 * Matrix6d::Identity();
      const Matrix7d N = Matrix7d::Identity() - J.transpose() * A.ldlt().solve(J);
      const Vector7d posture =
          config_.nullspace_kp * (scene.q_init[i] - state.arm(a).q) - config_.nullspace_kd * state.arm(a).dq;
      tau[i] = J.transpose() * w + model_->bias(a) + N * posture;

      const Vector6d grasp_error = poseError(grasp_targets[i], model_->eePose(a));
      max_position_error = std::max(max_position_error, grasp_error.head<3>().norm());
      max_orientation_error = std::max(max_orientation_error, grasp_error.tail<3>().norm());
      max_speed = std::max(max_speed, twist.head<3>().norm());
    }
    position_error_ = max_position_error;
    orientation_error_ = max_orientation_error;

    if (phase_ != Phase::Done && phase_ != Phase::Failed && t - start_time_ > config_.timeout)
      enterPhase(Phase::Failed, t);

    switch (phase_)
    {
    case Phase::Approach:
      if (t - phase_start_time_ >= config_.approach_time)
        enterPhase(Phase::Align, t);
      break;
    case Phase::Align:
      if (conditionHeld(max_position_error < config_.position_tolerance &&
                            max_orientation_error < config_.orientation_tolerance &&
                            max_speed < config_.speed_tolerance,
                        t, config_.align_hold_time))
        enterPhase(Phase::Close, t);
      break;
    case Phase::Close:
    {
      const double u = std::clamp((t - phase_start_time_) / config_.close_time, 0.0, 1.0);
      const double s = u * u * (3.0 - 2.0 * u);
      for (Arm a : kArms)
      {
        const GraspSpec &g = scene.graspOf(a);
        const double closed = std::max(0.0, g.finger_opening - g.contact_squeeze);
        finger_target_[armIndex(a)] = kOpenFinger + s * (closed - kOpenFinger);
      }
      bool gripped = u >= 1.0;
      for (const auto &arm : finger_normals)
        for (double f : arm)
          gripped &= f > config_.contact_force_min;
      if (conditionHeld(gripped, t, config_.settle_time))
        enterPhase(Phase::Done, t);
      break;
    }
    case Phase::Done:
    case Phase::Failed:
      break;
    }

    return {tau[armIndex(Arm::Left)], tau[armIndex(Arm::Right)]};
  }

} // namespace dual_arm
