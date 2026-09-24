#include "dual_arm/sim_env.hpp"

#include "dual_arm/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace dual_arm {
namespace {

Vector3d vec3(const mjtNum* v) { return Vector3d(v[0], v[1], v[2]); }

Matrix3d mat3(const mjtNum* xmat) {
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(xmat);
}

std::array<double, kNumArms> fingerOpenings(const SceneSpec& s) {
  return {s.grasp[0].finger_opening, s.grasp[1].finger_opening};
}

}  // namespace

SimEnv::SimEnv(const SimConfig& cfg) : cfg_(cfg), scene_(std::make_shared<SceneSpec>(cfg.scene)) {
  for (Arm a : kArms) base_true_[armIndex(a)] = plantBasePose(cfg_, a);
  if (cfg_.contact_grasp) {
    auto spec = loadSceneSpec(scene_->model_path, base_true_, {-1.0, -1.0}, cfg_.timestep);
    addContactGrippers(spec.get());
    if (scene_->name == "assembly") {
      // 平行夹爪绕指垫法向的抗扭只有 ~μ·N·r（接触斑半径 ~8 mm），远小于工件自重（偏心 0.1 m）
      // 与 5 N 预紧（力臂 0.2 m）产生的 ~1.7 N·m 俯仰力矩；悬空工件会绕指垫转到远端撞上工装。
      // 接触版在工装上加一块 10 mm 定位垫，让工件从一开始就平放在垫上（真实拧紧工位的做法），
      // 俯仰由定位垫承担，偏航（拧紧反力矩）仍由左手夹持与垫面摩擦共同承担。
      // 定位垫只在工件下方、远离左指（x < 0.05 m），控制器碰撞模型不必包含它。
      mjsGeom* jig = mjs_addGeom(mjs_findBody(spec.get(), "world"), nullptr);
      mjs_setName(jig->element, "contact_jig");
      jig->type = mjGEOM_BOX;
      jig->size[0] = 0.085;
      jig->size[1] = 0.022;
      jig->size[2] = 0.005;
      jig->pos[0] = -0.04;
      jig->pos[1] = -0.45;
      jig->pos[2] = 0.855;
      jig->friction[0] = 0.8;
      jig->rgba[0] = 0.25f;
      jig->rgba[1] = 0.45f;
      jig->rgba[2] = 0.70f;
      jig->rgba[3] = 1.0f;
    }
    // 临时托持工装：抓取建立前把主物体固定在世界系（相当于来料定位夹具），对准并夹紧后由 releaseStaging() 撤除。
    // 不能用 hand↔物体 的抓取 weld 代替：接近过程中手在运动，会把物体一起拖走。
    mjsEquality* staging = mjs_addEquality(spec.get(), nullptr);
    mjs_setName(staging->element, "contact_staging");
    staging->type = mjEQ_WELD;
    staging->objtype = mjOBJ_BODY;
    mjs_setString(staging->name1, "world");
    mjs_setString(staging->name2, scene_->object.body.c_str());
    // mjs_addEquality 的 data 默认是 joint 等式的 polycoef [0 1 0 …]，weld 会把它读成锚点 (0, 1, 0)；显式清零。
    // relpose 在 reset() 中按物体当前位姿写入；data[10] = torquescale。
    for (double& v : staging->data) v = 0.0;
    staging->data[6] = 1.0;
    staging->data[10] = 1.0;
    staging->solref[0] = 0.004;
    staging->solref[1] = 1.0;
    enableFingerGraspContacts(spec.get(), *scene_);
    model_ = compileSpec(spec.get(), scene_->model_path);
  } else {
    model_ = loadSceneModel(scene_->model_path, base_true_, fingerOpenings(*scene_), cfg_.timestep);
  }
  data_ = MjDataPtr(mj_makeData(model_.get()));
  if (!data_) throw std::runtime_error("mj_makeData failed");
  completeSceneSpec(*scene_, model_.get());
  idx_ = findSceneIndices(model_.get(), *scene_);
  if (cfg_.contact_grasp) {
    // completeSceneSpec 已把 contact_depth 计入 site_in_body；同步 plant 中的 site，使可视化与
    // TCP–抓取点误差统计看到同一个抓取点（控制器模型 / 碰撞模型经同一函数得到同一值）。
    for (Arm a : kArms) {
      const Pose& g = scene_->graspOf(a).site_in_body;
      const int site = idx_[a].grasp_site;
      for (int k = 0; k < 3; ++k) model_->site_pos[3 * site + k] = g.p[k];
    }
    staging_eq_ = mj_name2id(model_.get(), mjOBJ_EQUALITY, "contact_staging");
    if (staging_eq_ < 0) throw std::runtime_error("contact staging weld missing");
  }

  if (cfg_.contact_grasp) {
    for (Arm a : kArms) {
      const int i = armIndex(a);
      for (int f = 0; f < 2; ++f) {
        const std::string base = std::string(armName(a)) + (f == 0 ? "_left_finger" : "_right_finger");
        const int j = mj_name2id(model_.get(), mjOBJ_JOINT, (base + "_slide").c_str());
        finger_body_[i][f] = mj_name2id(model_.get(), mjOBJ_BODY, base.c_str());
        finger_act_[i][f] = mj_name2id(model_.get(), mjOBJ_ACTUATOR, (base + "_position").c_str());
        if (j < 0 || finger_body_[i][f] < 0 || finger_act_[i][f] < 0)
          throw std::runtime_error("contact gripper missing '" + base + "'");
        finger_qpos_[i][f] = model_->jnt_qposadr[j];
      }
    }
  }

  mjModel* m = model_.get();
  if (!cfg_.contacts) m->opt.disableflags |= mjDSBL_CONTACT;
  if (cfg_.contact_grasp) {
    // MuJoCo 的摩擦约束行没有位置项：持续切向载荷下软接触会以恒定速度蠕滑。
    // 金字塔锥 + 圆柱柄拧紧时手相对工具可转过 100° 以上；椭圆锥 + noslip 把它压到 < 1°。
    if (cfg_.contact_elliptic_cone) m->opt.cone = mjCONE_ELLIPTIC;
    m->opt.noslip_iterations = cfg_.contact_noslip_iterations;
  }
  if (cfg_.contact_grasp) {
    for (Arm a : kArms) {
      for (int f = 0; f < 2; ++f) {
        const int act = finger_act_[armIndex(a)][f];
        m->actuator_gainprm[mjNGAIN * act] = 3000.0;
        m->actuator_biasprm[mjNBIAS * act + 1] = -3000.0;
      }
    }
    for (int g = 0; g < m->ngeom; ++g) {
      for (Arm a : kArms) {
        const auto& fingers = finger_body_[armIndex(a)];
        if (m->geom_bodyid[g] == fingers[0] || m->geom_bodyid[g] == fingers[1])
          m->geom_friction[3 * g] = 1.5;
      }
    }
  }
  for (Arm a : kArms) {
    for (int j = 0; j < kArmDof; ++j) {
      const int act = idx_[a].actuator_id[j];
      tau_limit_[armIndex(a)][j] = std::min(std::abs(m->actuator_ctrlrange[2 * act]),
                                            std::abs(m->actuator_ctrlrange[2 * act + 1]));
    }
  }
  // 所有 rigid 约束（weld）使用 config 中的软约束参数
  for (int eq : idx_.rigid_welds) {
    for (int k = 0; k < mjNREF; ++k) m->eq_solref[mjNREF * eq + k] = cfg_.weld.solref[k];
    for (int k = 0; k < mjNIMP; ++k) m->eq_solimp[mjNIMP * eq + k] = cfg_.weld.solimp[k];
  }
  // 螺钉 hinge 的阻尼 / 摩擦
  if (idx_.screw.valid) {
    m->dof_damping[idx_.screw.hinge_dof] = scene_->screw.damping;
    m->dof_frictionloss[idx_.screw.hinge_dof] = scene_->screw.frictionloss;
  }
  // 扰动作用的 body
  for (const auto& d : cfg_.disturbances) {
    const std::string& name = d.body.empty() ? scene_->object.body : d.body;
    const int b = mj_name2id(m, mjOBJ_BODY, name.c_str());
    if (b < 0) throw std::runtime_error("disturbance body '" + name + "' not found");
    dist_body_.push_back(b);
  }
  reset();
}

void SimEnv::reset() {
  const mjModel* m = model_.get();
  mjData* d = data_.get();
  mj_resetData(m, d);
  finger_target_override_ = {-1.0, -1.0};

  for (Arm a : kArms) {
    for (int j = 0; j < kArmDof; ++j) d->qpos[idx_[a].qpos_adr + j] = scene_->q_init[armIndex(a)][j];
    if (cfg_.contact_grasp) {
      for (int f = 0; f < 2; ++f) {
        d->qpos[finger_qpos_[armIndex(a)][f]] = 0.04;
        d->ctrl[finger_act_[armIndex(a)][f]] = 0.04;
      }
    }
  }
  // 主物体放在第一个航点
  const Pose& T0 = scene_->waypoints.front().pose;
  mjtNum* qo = d->qpos + idx_.object_qpos_adr;
  qo[0] = T0.p.x();
  qo[1] = T0.p.y();
  qo[2] = T0.p.z();
  qo[3] = T0.q.w();
  qo[4] = T0.q.x();
  qo[5] = T0.q.y();
  qo[6] = T0.q.z();
  // 手里的工具（被抓 body 不是主物体时）：放到使抓取点与 TCP 重合的位置
  mj_kinematics(m, d);
  for (Arm a : kArms) {
    const auto& ai = idx_[a];
    if (ai.grasp_body == idx_.object_body) continue;
    const Pose T_tcp = poseFromMjMat(d->site_xpos + 3 * ai.ee_site, d->site_xmat + 9 * ai.ee_site);
    const Pose T_body = T_tcp * scene_->graspOf(a).site_in_body.inverse();
    mjtNum* qb = d->qpos + ai.grasp_qpos_adr;
    qb[0] = T_body.p.x();
    qb[1] = T_body.p.y();
    qb[2] = T_body.p.z();
    qb[3] = T_body.q.w();
    qb[4] = T_body.q.x();
    qb[5] = T_body.q.y();
    qb[6] = T_body.q.z();
  }
  // 接触模式：工具按抓取构型放好后，两臂移到预抓取构型（由抓取对准求出）
  if (cfg_.contact_grasp && start_q_set_) {
    for (Arm a : kArms) {
      for (int j = 0; j < kArmDof; ++j) d->qpos[idx_[a].qpos_adr + j] = start_q_[armIndex(a)][j];
    }
  }

  mj_forward(m, d);
  for (Arm a : kArms) {
    const auto& ai = idx_[a];
    const Pose es = poseFromMjMat(d->site_xpos + 3 * ai.ee_site, d->site_xmat + 9 * ai.ee_site);
    const Pose gs = poseFromMjMat(d->site_xpos + 3 * ai.grasp_site, d->site_xmat + 9 * ai.grasp_site);
    init_mismatch_[armIndex(a)] = poseError(gs, es);
  }
  initWelds();
  if (cfg_.contact_grasp) {
    // hand↔物体 的抓取 weld 在接触 plant 中始终关闭；世界系临时托持工装锁住物体当前位姿。
    for (Arm a : kArms) d->eq_active[idx_[a].weld_eq] = 0;
    const Pose object = poseFromMjMat(d->xpos + 3 * idx_.object_body, d->xmat + 9 * idx_.object_body);
    writeRelpose(staging_eq_, object.p, object.q);
    d->eq_active[staging_eq_] = 1;
  }
  mj_forward(m, d);
  updateFingerContacts();

  tau_seat_ = 0.0;
  state_ = DualArmState{};
  readKinematics(state_);
  readForces(state_);  // t = 0 的力来自零力矩下的 mj_forward，仅作为初值
  last_step_ = state_;
}

void SimEnv::forward() {
  mj_forward(model_.get(), data_.get());
  updateFingerContacts();
  readKinematics(state_);
  readForces(state_);
  last_step_ = state_;
}

void SimEnv::initWelds() {
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  // eq_data 布局（body 语义的 weld）：
  //   [0:3]  anchor：约束点在 body2 坐标系中的位置
  //   [3:6]  relpose 位置：同一点在 body1 坐标系中的期望位置
  //   [6:10] relpose 姿态：body2 相对 body1 的期望姿态 q_1^{-1} ⊗ q_2 (w,x,y,z)
  // 约束方程：x_1 + R_1·relpos = x_2 + R_2·anchor，q_1 ⊗ relquat = q_2
  // 所有 rigid 约束先按“当前实际相对位姿”初始化 → t=0 约束误差为 0，无预载内力
  for (int eq : idx_.rigid_welds) {
    const int b1 = m->eq_obj1id[eq], b2 = m->eq_obj2id[eq];
    const Vector3d anchor = vec3(m->eq_data + mjNEQDATA * eq);
    const Pose T1 = poseFromMj(d->xpos + 3 * b1, d->xquat + 4 * b1);
    const Pose T2 = poseFromMj(d->xpos + 3 * b2, d->xquat + 4 * b2);
    writeRelpose(eq, T1.inverse().transformPoint(T2.transformPoint(anchor)), (T1.q.conjugate() * T2.q).normalized());
  }
  // 夹爪 weld：anchor 设为抓取点；记录 current / nominal 两种 relpose
  for (Arm a : kArms) {
    const auto& ai = idx_[a];
    const int i = armIndex(a);
    const Pose& T_gs = scene_->graspOf(a).site_in_body;  // 抓取点在被抓 body 系中
    mjtNum* e = model_->eq_data + mjNEQDATA * ai.weld_eq;
    for (int k = 0; k < 3; ++k) e[k] = T_gs.p[k];
    const Pose T_hand = poseFromMj(d->xpos + 3 * ai.hand_body, d->xquat + 4 * ai.hand_body);
    const Pose T_body = poseFromMj(d->xpos + 3 * ai.grasp_body, d->xquat + 4 * ai.grasp_body);
    relpos_current_[i] = T_hand.inverse().transformPoint(T_body.transformPoint(T_gs.p));
    relquat_current_[i] = (T_hand.q.conjugate() * T_body.q).normalized();
    // 名义几何：ee_site 与抓取点完全重合（T_es = ee_site 在 hand 系中的位姿）
    const Pose T_es = poseFromMj(m->site_pos + 3 * ai.ee_site, m->site_quat + 4 * ai.ee_site);
    relpos_nominal_[i] = T_es.p;
    relquat_nominal_[i] = (T_es.q * T_gs.q.conjugate()).normalized();
  }
  const bool nominal = cfg_.weld.init_mode == WeldConfig::InitMode::Nominal;
  ramp_active_ = nominal && cfg_.weld.nominal_ramp_time > 0.0;
  for (Arm a : kArms) {
    const int i = armIndex(a);
    if (nominal && !ramp_active_) {
      writeRelpose(idx_[a].weld_eq, relpos_nominal_[i], relquat_nominal_[i]);
    } else {
      writeRelpose(idx_[a].weld_eq, relpos_current_[i], relquat_current_[i]);
    }
  }
}

void SimEnv::writeRelpose(int eq, const Vector3d& pos, const Quaterniond& quat) {
  mjtNum* e = model_->eq_data + mjNEQDATA * eq;
  for (int k = 0; k < 3; ++k) e[3 + k] = pos[k];
  e[6] = quat.w();
  e[7] = quat.x();
  e[8] = quat.y();
  e[9] = quat.z();
}

void SimEnv::updateWeldRamp(double t) {
  if (!ramp_active_) return;
  const double tau = std::min(t / cfg_.weld.nominal_ramp_time, 1.0);
  const double s = tau * tau * (3.0 - 2.0 * tau);  // smoothstep：两端速度为 0
  for (Arm a : kArms) {
    const int i = armIndex(a);
    writeRelpose(idx_[a].weld_eq, (1.0 - s) * relpos_current_[i] + s * relpos_nominal_[i],
                 relquat_current_[i].slerp(s, relquat_nominal_[i]));
  }
  if (tau >= 1.0) ramp_active_ = false;
}

void SimEnv::applyDisturbances(double t) {
  mjData* d = data_.get();
  for (int b : dist_body_) {
    for (int k = 0; k < 6; ++k) d->xfrc_applied[6 * b + k] = 0.0;
  }
  for (std::size_t i = 0; i < cfg_.disturbances.size(); ++i) {
    const auto& dc = cfg_.disturbances[i];
    const double s = dc.profile(t);
    if (s == 0.0) continue;
    const int b = dist_body_[i];
    Vector3d f = s * dc.force, tq = s * dc.torque;
    if (dc.frame == DisturbanceConfig::Frame::Body) {
      const Matrix3d R = mat3(d->xmat + 9 * b);
      f = R * f;
      tq = R * tq;
    }
    mjtNum* xf = d->xfrc_applied + 6 * b;  // 布局：[force(3), torque(3)]，世界系，作用于质心
    for (int k = 0; k < 3; ++k) {
      xf[k] += f[k];
      xf[3 + k] += tq[k];
    }
  }
}

void SimEnv::applyScrewSeatTorque() {
  if (!idx_.screw.valid) return;
  mjData* d = data_.get();
  const ScrewParams& p = scene_->screw;
  const double feed = -d->qpos[idx_.screw.slide_qpos];
  const double pen = feed - p.seat_depth;
  // 座面贴合后，阻力矩随拧入深度线性增大（沿 +z，阻碍继续拧紧）；弹性的，松开时会推回
  tau_seat_ = pen > 0.0 ? std::min(p.seat_stiffness * pen, p.seat_torque_max) : 0.0;
  // 接触版装配使用自锁螺纹近似：座面只抵抗继续旋入，不在松手时
  // 主动把螺钉和工具弹回。旧 weld 基线仍沿用弹性座面模型。
  d->qfrc_applied[idx_.screw.hinge_dof] =
      cfg_.contact_grasp && d->qvel[idx_.screw.hinge_dof] >= 0.0 ? 0.0 : tau_seat_;
}

void SimEnv::step(const Vector7d& tau_left, const Vector7d& tau_right) {
  const mjModel* m = model_.get();
  mjData* d = data_.get();

  // 本步记录从当前观测（t 时刻运动学）开始
  last_step_ = state_;
  for (Arm a : kArms) {
    const Vector7d& tau_in = (a == Arm::Left) ? tau_left : tau_right;
    const Vector7d& lim = tau_limit_[armIndex(a)];
    Vector7d tau = tau_in.cwiseMax(-lim).cwiseMin(lim);
    for (int j = 0; j < kArmDof; ++j) {
      if (!std::isfinite(tau[j])) tau[j] = 0.0;  // 防止控制器输出 NaN 直接毁掉仿真
      d->ctrl[idx_[a].actuator_id[j]] = tau[j];
    }
    last_step_.arm(a).tau = tau;
    if (cfg_.contact_grasp) {
      // 手指开度由抓取对准（GraspAlignment）经 setFingerTarget 指令；未指令时保持全开。
      const double opening = finger_target_override_[armIndex(a)] >= 0.0 ? finger_target_override_[armIndex(a)] : 0.04;
      for (int f = 0; f < 2; ++f) d->ctrl[finger_act_[armIndex(a)][f]] = opening;
    }
  }
  updateWeldRamp(d->time);
  applyDisturbances(d->time);
  applyScrewSeatTorque();
  last_step_.disturbance = disturbanceAt(d->time);

  // 加速度 / 约束力 / F/T（t 时刻）+ 积分到 t+dt
  mj_step2(m, d);
  readForces(last_step_);
  updateFingerContacts();

  // t+dt 时刻的位置 / 速度相关量
  mj_step1(m, d);
  readKinematics(state_);
  for (Arm a : kArms) {
    ArmState& s = state_.arm(a);
    const ArmState& r = last_step_.arm(a);
    s.tau = r.tau;
    s.ft_raw = r.ft_raw;
    s.ft_ee_world = r.ft_ee_world;
    s.weld_wrench = r.weld_wrench;
  }
  state_.disturbance = last_step_.disturbance;
  state_.screw.tau_seat = last_step_.screw.tau_seat;
  state_.screw.tau_friction = last_step_.screw.tau_friction;
}

void SimEnv::readKinematics(DualArmState& s) const {
  const mjData* d = data_.get();
  s.t = d->time;
  for (Arm a : kArms) {
    const auto& ai = idx_[a];
    ArmState& as = s.arm(a);
    as.q = Eigen::Map<const Vector7d>(d->qpos + ai.qpos_adr);
    as.dq = Eigen::Map<const Vector7d>(d->qvel + ai.dof_adr);
    as.ee_pose = eePose(a);
    as.ee_twist = eeTwist(a);
  }
  s.object = objectState();
  const ScrewState ss = screwState();
  s.screw.valid = ss.valid;
  s.screw.angle = ss.angle;
  s.screw.rate = ss.rate;
  s.screw.feed = ss.feed;
  s.screw.lead_error = ss.lead_error;
  s.screw.tau_damping = ss.tau_damping;
}

void SimEnv::readForces(DualArmState& s) const {
  for (Arm a : kArms) {
    ArmState& as = s.arm(a);
    as.ft_raw = wristFtRaw(a);
    as.ft_ee_world = wristWrenchOnObject(a);
    as.weld_wrench = weldWrench(a);
  }
  if (idx_.screw.valid) {
    const mjData* d = data_.get();
    s.screw.tau_seat = tau_seat_;
    s.screw.tau_friction = 0.0;
    for (int r = 0; r < d->nefc; ++r) {
      if (d->efc_type[r] == mjCNSTR_FRICTION_DOF && d->efc_id[r] == idx_.screw.hinge_dof) {
        s.screw.tau_friction = d->efc_force[r];
      }
    }
  }
}

Pose SimEnv::eePose(Arm a) const {
  const int s = idx_[a].ee_site;
  return poseFromMjMat(data_->site_xpos + 3 * s, data_->site_xmat + 9 * s);
}

Twist SimEnv::eeTwist(Arm a) const {
  mjtNum rotlin[6];
  // flg_local = 0：世界系表达，参考点 = site 原点；输出顺序为 [ω; v]
  mj_objectVelocity(model_.get(), data_.get(), mjOBJ_SITE, idx_[a].ee_site, rotlin, 0);
  Twist v;
  v << rotlin[3], rotlin[4], rotlin[5], rotlin[0], rotlin[1], rotlin[2];
  return v;
}

Wrench SimEnv::wristFtRaw(Arm a) const {
  const auto& ai = idx_[a];
  Wrench w;
  w.head<3>() = vec3(data_->sensordata + ai.ft_force_adr);
  w.tail<3>() = vec3(data_->sensordata + ai.ft_torque_adr);
  if (cfg_.sensors.fix_weld_torque) {
    const Matrix3d R_s = mat3(data_->site_xmat + 9 * ai.ft_site);
    w.tail<3>() += R_s.transpose() * weldTorqueCorrection(a);
  }
  return w;
}

int SimEnv::weldFirstRow(int eq) const {
  const mjData* d = data_.get();
  for (int r = 0; r < d->nefc; ++r) {
    if (d->efc_type[r] == mjCNSTR_EQUALITY && d->efc_id[r] == eq) return r;
  }
  return -1;
}

Vector3d SimEnv::weldTorqueCorrection(Arm a) const {
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  const auto& ai = idx_[a];
  const int r0 = weldFirstRow(ai.weld_eq);
  if (r0 < 0) return Vector3d::Zero();  // weld 未激活
  // MuJoCo 的解释：转动行约束力直接作为施加给 body1(hand) 的世界系力矩
  const Vector3d f_rot(d->efc_force[r0 + 3], d->efc_force[r0 + 4], d->efc_force[r0 + 5]);
  // 真实作用：转动行的 Jᵀf 在被抓 body 转动自由度上的广义力（body 局部系力矩），换到世界系
  const int nv = m->nv;
  const int b0 = ai.grasp_dof_adr + 3;
  const bool sparse = mj_isSparse(m);
  Vector3d tau_local = Vector3d::Zero();
  for (int r = r0 + 3; r < r0 + 6; ++r) {
    const double f = d->efc_force[r];
    if (!sparse) {
      const mjtNum* row = d->efc_J + static_cast<long>(r) * nv;
      for (int k = 0; k < 3; ++k) tau_local[k] += row[b0 + k] * f;
    } else {
      const int adr = d->efc_J_rowadr[r];
      for (int n = 0; n < d->efc_J_rownnz[r]; ++n) {
        const int col = d->efc_J_colind[adr + n];
        if (col >= b0 && col < b0 + 3) tau_local[col - b0] += d->efc_J[adr + n] * f;
      }
    }
  }
  const Vector3d tau_rot_body = mat3(d->xmat + 9 * ai.grasp_body) * tau_local;
  // cfrc_int_true = cfrc_int_mujoco + ext_mujoco − ext_true，ext_true(hand) = −tau_rot_body
  return f_rot + tau_rot_body;
}

Wrench SimEnv::wristWrenchOnObject(Arm a) const {
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  const auto& ai = idx_[a];
  const Wrench raw = wristFtRaw(a);
  const Matrix3d R_s = mat3(d->site_xmat + 9 * ai.ft_site);
  const Vector3d p_ft = vec3(d->site_xpos + 3 * ai.ft_site);
  const Vector3d p_ee = vec3(d->site_xpos + 3 * ai.ee_site);
  // 传感器测的是 link7 对整个 hand 子树（hand + 两根固定手指）的力，所以用子树质量与子树质心
  const Vector3d p_com = vec3(d->subtree_com + 3 * ai.hand_body);
  const Vector3d mg = m->body_subtreemass[ai.hand_body] * vec3(m->opt.gravity);

  // hand 子树静力平衡：F_link7→hand + F_obj→hand + m·g = 0
  //   ⇒ 夹爪施加给物体的力  F = −F_obj→hand = F_link7→hand + m·g
  //   ⇒ 对 ee_site（TCP）的力矩  M = M_link7→hand(ee_site) + (p_com − p_ee) × m·g
  const Vector3d F_l7 = R_s * raw.head<3>();
  const Vector3d M_l7_ft = R_s * raw.tail<3>();
  Wrench w;
  w.head<3>() = F_l7 + mg;
  w.tail<3>() = M_l7_ft + (p_ft - p_ee).cross(F_l7) + (p_com - p_ee).cross(mg);
  return w;
}

Wrench SimEnv::constraintWrenchOnBody(int eq, int body) const {
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  const int j = m->body_jntadr[body];
  const int nv = m->nv;
  const int b0 = m->jnt_dofadr[j];
  const bool sparse = mj_isSparse(m);

  // 广义力 g = Σ_rows J[row, body dofs]ᵀ · efc_force[row]
  double g[6] = {0, 0, 0, 0, 0, 0};
  for (int r = 0; r < d->nefc; ++r) {
    if (d->efc_type[r] != mjCNSTR_EQUALITY || d->efc_id[r] != eq) continue;
    const double f = d->efc_force[r];
    if (!sparse) {
      const mjtNum* row = d->efc_J + static_cast<long>(r) * nv;
      for (int k = 0; k < 6; ++k) g[k] += row[b0 + k] * f;
    } else {
      const int adr = d->efc_J_rowadr[r];
      for (int n = 0; n < d->efc_J_rownnz[r]; ++n) {
        const int col = d->efc_J_colind[adr + n];
        if (col >= b0 && col < b0 + 6) g[col - b0] += d->efc_J[adr + n] * f;
      }
    }
  }
  // 自由关节：平动广义力 = 世界系力；转动广义力 = 绕 body 原点的力矩，表达在 body 局部系
  Wrench w;
  w.head<3>() = Vector3d(g[0], g[1], g[2]);
  w.tail<3>() = mat3(d->xmat + 9 * body) * Vector3d(g[3], g[4], g[5]);
  return w;
}

Wrench SimEnv::weldWrench(Arm a) const {
  const auto& ai = idx_[a];
  const mjData* d = data_.get();
  const Wrench w_origin = constraintWrenchOnBody(ai.weld_eq, ai.grasp_body);
  return shiftWrenchRefPoint(w_origin, vec3(d->xpos + 3 * ai.grasp_body), vec3(d->site_xpos + 3 * ai.grasp_site));
}

double SimEnv::fingerContactNormal(Arm a, int finger) const {
  if (!cfg_.contact_grasp || finger < 0 || finger > 1) return 0.0;
  return finger_normal_[armIndex(a)][finger];
}

double SimEnv::fingerContactTangent(Arm a, int finger) const {
  if (!cfg_.contact_grasp || finger < 0 || finger > 1) return 0.0;
  return finger_tangent_[armIndex(a)][finger];
}

double SimEnv::fingerContactMu(Arm a, int finger) const {
  if (!cfg_.contact_grasp || finger < 0 || finger > 1) return 0.0;
  return finger_mu_[armIndex(a)][finger];
}

void SimEnv::updateFingerContacts() {
  for (auto& arm : finger_normal_) arm = {0.0, 0.0};
  for (auto& arm : finger_tangent_) arm = {0.0, 0.0};
  for (auto& arm : finger_mu_) arm = {0.0, 0.0};
  if (!cfg_.contact_grasp) return;
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  for (int c = 0; c < d->ncon; ++c) {
    const mjContact& contact = d->contact[c];
    if (contact.efc_address < 0) continue;
    const int b1 = m->geom_bodyid[contact.geom[0]];
    const int b2 = m->geom_bodyid[contact.geom[1]];
    for (Arm a : kArms) {
      const int target = idx_[a].grasp_body;
      for (int f = 0; f < 2; ++f) {
        const int fb = finger_body_[armIndex(a)][f];
        if (!((b1 == fb && b2 == target) || (b2 == fb && b1 == target))) continue;
        mjtNum wrench[6];
        mj_contactForce(m, d, c, wrench);
        finger_normal_[armIndex(a)][f] += std::abs(wrench[0]);
        finger_tangent_[armIndex(a)][f] += std::hypot(wrench[1], wrench[2]);
        finger_mu_[armIndex(a)][f] = std::max(finger_mu_[armIndex(a)][f], contact.friction[0]);
      }
    }
  }
}

double SimEnv::fingerOpening(Arm a, int finger) const {
  if (!cfg_.contact_grasp || finger < 0 || finger > 1) return 0.0;
  return data_->qpos[finger_qpos_[armIndex(a)][finger]];
}

void SimEnv::setStartConfiguration(const std::array<Vector7d, kNumArms>& q) {
  start_q_ = q;
  start_q_set_ = true;
}

void SimEnv::releaseStaging() {
  if (cfg_.contact_grasp) data_->eq_active[staging_eq_] = 0;
}

bool SimEnv::stagingActive() const {
  return cfg_.contact_grasp && data_->eq_active[staging_eq_] != 0;
}

void SimEnv::setFingerTarget(Arm a, double opening) {
  if (!cfg_.contact_grasp) return;
  finger_target_override_[armIndex(a)] = opening < 0.0
      ? -1.0 : std::clamp(opening, 0.0, 0.04);
}

ObjectState SimEnv::objectState() const {
  const mjData* d = data_.get();
  ObjectState o;
  o.pose = poseFromMj(d->qpos + idx_.object_qpos_adr, d->qpos + idx_.object_qpos_adr + 3);
  const mjtNum* v = d->qvel + idx_.object_dof_adr;
  o.twist.head<3>() = Vector3d(v[0], v[1], v[2]);             // 世界系线速度（body 原点 = 中心）
  o.twist.tail<3>() = o.pose.R() * Vector3d(v[3], v[4], v[5]);  // 局部角速度 → 世界系
  return o;
}

ScrewState SimEnv::screwState() const {
  ScrewState s;
  if (!idx_.screw.valid) return s;
  const mjData* d = data_.get();
  s.valid = true;
  s.angle = d->qpos[idx_.screw.hinge_qpos];
  s.rate = d->qvel[idx_.screw.hinge_dof];
  s.feed = -d->qpos[idx_.screw.slide_qpos];
  if (const ClosedChainConstraint* c = scene_->screwConstraint()) {
    s.lead_error = d->qpos[idx_.screw.slide_qpos] - c->lead / (2.0 * M_PI) * s.angle;
  }
  s.tau_seat = tau_seat_;
  s.tau_damping = -model_->dof_damping[idx_.screw.hinge_dof] * s.rate;
  return s;
}

Wrench SimEnv::disturbanceAt(double t) const {
  Wrench w = Wrench::Zero();
  const mjData* d = data_.get();
  for (std::size_t i = 0; i < cfg_.disturbances.size(); ++i) {
    if (dist_body_[i] != idx_.object_body) continue;
    const auto& dc = cfg_.disturbances[i];
    const double s = dc.profile(t);
    if (s == 0.0) continue;
    Vector3d f = s * dc.force, tq = s * dc.torque;
    if (dc.frame == DisturbanceConfig::Frame::Body) {
      const Matrix3d R = mat3(d->xmat + 9 * idx_.object_body);
      f = R * f;
      tq = R * tq;
    }
    w.head<3>() += f;
    w.tail<3>() += tq;
  }
  return w;
}

bool SimEnv::diverged() const {
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  for (int i = 0; i < m->nq; ++i) {
    if (!std::isfinite(d->qpos[i])) return true;
  }
  for (int i = 0; i < m->nv; ++i) {
    if (!std::isfinite(d->qvel[i])) return true;
  }
  return d->warning[mjWARN_BADQACC].number > 0 || d->warning[mjWARN_BADQPOS].number > 0 ||
         d->warning[mjWARN_BADQVEL].number > 0;
}

}  // namespace dual_arm
