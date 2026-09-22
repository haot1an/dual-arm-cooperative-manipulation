#include "dual_arm/mujoco_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dual_arm {

MujocoRobotModel::MujocoRobotModel(const SimConfig& cfg) : scene_(cfg.scene) {
  for (Arm a : kArms) base_[armIndex(a)] = nominalBasePose(cfg, a);
  const std::array<double, kNumArms> fingers{scene_.grasp[0].finger_opening, scene_.grasp[1].finger_opening};
  model_ = loadSceneModel(scene_.model_path, base_, fingers, cfg.timestep);
  data_ = MjDataPtr(mj_makeData(model_.get()));
  if (!data_) throw std::runtime_error("mj_makeData failed");
  completeSceneSpec(scene_, model_.get());
  idx_ = findSceneIndices(model_.get(), scene_);

  const mjModel* m = model_.get();
  const int nv = m->nv;
  jacp_.assign(3 * nv, 0.0);
  jacr_.assign(3 * nv, 0.0);
  jacp_dot_.assign(3 * nv, 0.0);
  jacr_dot_.assign(3 * nv, 0.0);
  full_m_.assign(static_cast<std::size_t>(nv) * nv, 0.0);
  bias_.assign(nv, 0.0);
  grav_.assign(nv, 0.0);

  for (Arm a : kArms) {
    const int i = armIndex(a);
    for (int j = 0; j < kArmDof; ++j) {
      const int jid = idx_[a].joint_id[j];
      q_lo_[i][j] = m->jnt_range[2 * jid];
      q_hi_[i][j] = m->jnt_range[2 * jid + 1];
      const int act = idx_[a].actuator_id[j];
      tau_lim_[i][j] = std::min(std::abs(m->actuator_ctrlrange[2 * act]),
                                std::abs(m->actuator_ctrlrange[2 * act + 1]));
    }
  }

  // 物体 / 工具的 qpos 与本模型的计算无关（两臂的运动学树不含它们），只需合法
  mj_resetData(m, data_.get());
  DualArmState s0;
  for (Arm a : kArms) s0.arm(a).q = scene_.q_init[armIndex(a)];
  update(s0);
}

void MujocoRobotModel::update(const DualArmState& state) {
  const mjModel* m = model_.get();
  mjData* d = data_.get();
  const int nv = m->nv;

  for (Arm a : kArms) {
    const auto& ai = idx_[a];
    Eigen::Map<Vector7d>(d->qpos + ai.qpos_adr) = state.arm(a).q;
    Eigen::Map<Vector7d>(d->qvel + ai.dof_adr).setZero();
  }
  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_crb(m, d);
  mj_fullM(m, d, full_m_.data());

  // 重力项：qvel = 0 时的偏置力
  mj_comVel(m, d);
  mj_rne(m, d, 0, grav_.data());

  // 偏置项：C·dq + g
  for (Arm a : kArms) {
    Eigen::Map<Vector7d>(d->qvel + idx_[a].dof_adr) = state.arm(a).dq;
  }
  mj_comVel(m, d);
  mj_rne(m, d, 0, bias_.data());

  using RowMat3 = Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>;
  using RowMatN = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  const Eigen::Map<const RowMatN> Mfull(full_m_.data(), nv, nv);

  for (Arm a : kArms) {
    const auto& ai = idx_[a];
    ArmCache& c = cache_[armIndex(a)];
    const int s = ai.ee_site;
    const int d0 = ai.dof_adr;

    c.ee = poseFromMjMat(d->site_xpos + 3 * s, d->site_xmat + 9 * s);

    mj_jacSite(m, d, jacp_.data(), jacr_.data(), s);
    const Eigen::Map<const RowMat3> Jp(jacp_.data(), 3, nv);
    const Eigen::Map<const RowMat3> Jr(jacr_.data(), 3, nv);
    c.J.topRows<3>() = Jp.middleCols<kArmDof>(d0);
    c.J.bottomRows<3>() = Jr.middleCols<kArmDof>(d0);

    mj_jacDot(m, d, jacp_dot_.data(), jacr_dot_.data(), d->site_xpos + 3 * s, m->site_bodyid[s]);
    const Eigen::Map<const RowMat3> Jpd(jacp_dot_.data(), 3, nv);
    const Eigen::Map<const RowMat3> Jrd(jacr_dot_.data(), 3, nv);
    const Vector7d& dq = state.arm(a).dq;
    c.Jdot_dq.head<3>() = Jpd.middleCols<kArmDof>(d0) * dq;
    c.Jdot_dq.tail<3>() = Jrd.middleCols<kArmDof>(d0) * dq;

    c.twist = c.J * dq;
    c.M = Mfull.block<kArmDof, kArmDof>(d0, d0);
    c.h = Eigen::Map<const Vector7d>(bias_.data() + d0);
    c.g = Eigen::Map<const Vector7d>(grav_.data() + d0);
  }
}

}  // namespace dual_arm
