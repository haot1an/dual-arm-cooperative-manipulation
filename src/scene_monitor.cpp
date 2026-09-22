#include "dual_arm/scene_monitor.hpp"

#include "dual_arm/math_utils.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace dual_arm {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const char* const kFixedColumns[] = {
    "grasp_squeeze", "grasp_twist",     "roll_obj",        "roll_l",         "roll_r",         "roll_sync",
    "obj_drift_x",   "obj_drift_y",     "obj_drift_z",     "obj_drift_rx",   "obj_drift_ry",   "obj_drift_rz",
    "screw_angle",   "screw_rate",      "screw_feed",      "screw_tau_seat", "screw_tau_damp", "screw_tau_fric",
    "screw_tau_resist", "screw_lead_err", "hold_ft_axis", "hold_weld_axis",  "work_ft_axis",   "work_weld_axis",
    "hold_ft_axial", "hold_weld_axial", "work_ft_axial", "work_weld_axial", "d_min"};
constexpr int kNumFixed = sizeof(kFixedColumns) / sizeof(kFixedColumns[0]);

Matrix3d mat3(const mjtNum* xmat) { return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(xmat); }

}  // namespace

SceneMonitor::SceneMonitor(const SimEnv& env, bool with_distances) : env_(env) {
  const SceneSpec& sc = env.scene();
  const SimConfig& cfg = env.config();
  if (with_distances && !sc.obstacles.empty()) {
    std::array<Pose, kNumArms> base{env.basePose(Arm::Left), env.basePose(Arm::Right)};
    collision_ = std::make_unique<CollisionModel>(sc, base, cfg.collision, env.timestep());
  }
  same_body_ = sc.grasp[0].body == sc.grasp[1].body;
  hold_ = cfg.asym_coop.holding_arm;
  work_ = hold_ == Arm::Left ? Arm::Right : Arm::Left;
  const mjModel* m = env.model();
  if (const ClosedChainConstraint* c = sc.screwConstraint()) {
    axis_body_ = mj_name2id(m, mjOBJ_BODY, c->body1.c_str());
    axis_local_ = c->axis.normalized();
    point_local_ = c->point;
  } else {
    axis_body_ = env.indices().object_body;
  }
  for (const char* n : kFixedColumns) names_.emplace_back(n);
  if (collision_) {
    for (int g = 0; g < collision_->numGroups(); ++g) names_.push_back("d_" + collision_->groupName(g));
  }
  values_.assign(names_.size(), kNaN);
  reset(env.state());
}

void SceneMonitor::reset(const DualArmState& s0) {
  obj0_ = s0.object.pose;
  for (Arm a : kArms) tcp0_[armIndex(a)] = s0.arm(a).ee_pose;
  const mjData* d = env_.data();
  const auto& idx = env_.indices();
  const Vector3d pL(d->site_xpos + 3 * idx[Arm::Left].grasp_site);
  const Vector3d pR(d->site_xpos + 3 * idx[Arm::Right].grasp_site);
  e0_ = (pR - pL).norm() > 1e-9 ? Vector3d((pR - pL).normalized()) : Vector3d::UnitX();
}

void SceneMonitor::update(const DualArmState& rec) {
  const mjData* d = env_.data();
  const auto& idx = env_.indices();
  double* v = values_.data();
  for (double& x : values_) x = kNaN;

  // --- 抓取连线上的挤压力 / 扭转力矩（weld 真值），两手抓同一 body 时才有意义 ---
  const Vector3d pL(d->site_xpos + 3 * idx[Arm::Left].grasp_site);
  const Vector3d pR(d->site_xpos + 3 * idx[Arm::Right].grasp_site);
  if (same_body_ && (pR - pL).norm() > 1e-9) {
    const Vector3d e = (pR - pL).normalized();
    const Wrench& wL = rec.arm(Arm::Left).weld_wrench;
    const Wrench& wR = rec.arm(Arm::Right).weld_wrench;
    v[0] = 0.5 * (wL.head<3>() - wR.head<3>()).dot(e);
    v[1] = 0.5 * (wL.tail<3>() - wR.tail<3>()).dot(e);
  }
  // --- 滚转同步 ---
  v[2] = rotationError(rec.object.pose.R(), obj0_.R()).dot(e0_);
  v[3] = rotationError(rec.arm(Arm::Left).ee_pose.R(), tcp0_[0].R()).dot(e0_);
  v[4] = rotationError(rec.arm(Arm::Right).ee_pose.R(), tcp0_[1].R()).dot(e0_);
  v[5] = v[3] - v[4];
  // --- 物体漂移（物体初始坐标系）---
  const Vector6d drift = decomposePoseDrift(obj0_, rec.object.pose, obj0_.R());
  for (int k = 0; k < 6; ++k) v[6 + k] = drift[k];
  // --- 螺钉 ---
  if (rec.screw.valid) {
    v[12] = rec.screw.angle;
    v[13] = rec.screw.rate;
    v[14] = rec.screw.feed;
    v[15] = rec.screw.tau_seat;
    v[16] = rec.screw.tau_damping;
    v[17] = rec.screw.tau_friction;
    v[18] = rec.screw.tauResist();
    v[19] = rec.screw.lead_error;
  }
  // --- 绕任务轴的力矩：m_P·a，P 为轴上一点；wrench 参考点换到 P ---
  if (axis_body_ >= 0) {
    const Matrix3d R = mat3(d->xmat + 9 * axis_body_);
    const Vector3d a = R * axis_local_;
    const Vector3d P = Vector3d(d->xpos + 3 * axis_body_) + R * point_local_;
    auto aboutAxis = [&](const Wrench& w, const Vector3d& ref) { return shiftWrenchRefPoint(w, ref, P).tail<3>().dot(a); };
    const auto& h = rec.arm(hold_);
    const auto& wk = rec.arm(work_);
    const Vector3d gh(d->site_xpos + 3 * idx[hold_].grasp_site), gw(d->site_xpos + 3 * idx[work_].grasp_site);
    v[20] = aboutAxis(h.ft_ee_world, h.ee_pose.p);
    v[21] = aboutAxis(h.weld_wrench, gh);
    v[22] = aboutAxis(wk.ft_ee_world, wk.ee_pose.p);
    v[23] = aboutAxis(wk.weld_wrench, gw);
    v[24] = h.ft_ee_world.head<3>().dot(a);
    v[25] = h.weld_wrench.head<3>().dot(a);
    v[26] = wk.ft_ee_world.head<3>().dot(a);
    v[27] = wk.weld_wrench.head<3>().dot(a);
  }
  // --- 最小距离 ---
  if (collision_) {
    collision_->query(rec.arm(Arm::Left).q, rec.arm(Arm::Right).q);
    v[28] = collision_->minDistance();
    for (int g = 0; g < collision_->numGroups(); ++g) v[kNumFixed + g] = collision_->groupMinDistance(g);
  }
}

double SceneMonitor::value(const std::string& column) const {
  for (std::size_t i = 0; i < names_.size(); ++i) {
    if (names_[i] == column) return values_[i];
  }
  throw std::out_of_range("SceneMonitor: no column '" + column + "'");
}

}  // namespace dual_arm
