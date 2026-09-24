// 独立的双臂接触夹取验证：可动手指 + 摩擦接触 + 无 weld 的抬起。
// 不复用旧 SimEnv：旧闭链控制器的 SceneSpec 明确要求永久 weld。
#include "dual_arm/config.hpp"
#include "dual_arm/mujoco_utils.hpp"
#ifdef DUAL_ARM_HAS_VIEWER
#include "dual_arm/viewer.hpp"
#endif

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int id(const mjModel* m, mjtObj kind, const std::string& name) {
  const int value = mj_name2id(m, kind, name.c_str());
  if (value < 0) throw std::runtime_error("missing " + name);
  return value;
}

struct Arm {
  std::array<int, 7> q{}, v{}, act{};
  std::array<int, 2> finger_q{}, finger_act{}, finger_body{};
  int site = -1;
  Eigen::Matrix<double, 7, 1> initial_q, lift_dq;
};

double smooth(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    bool headless = false, check = false;
    std::string camera = "cam_front";
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--headless") headless = true;
      else if (arg == "--check") check = true;
      else if (arg == "--camera" && i + 1 < argc) camera = argv[++i];
      else throw std::runtime_error("usage: run_contact_grasp [--headless] [--check] [--camera cam_front|cam_iso]");
    }

    const auto cfg = dual_arm::loadConfig("config/default.yaml", {}, "lift");
    const std::string path = cfg.root_dir + "/models/scene_contact_grasp.xml";
    const std::array<dual_arm::Pose, 2> bases = {
        dual_arm::nominalBasePose(cfg, dual_arm::Arm::Left),
        dual_arm::nominalBasePose(cfg, dual_arm::Arm::Right)};
    auto spec = dual_arm::loadSceneSpec(path, bases, {-1.0, -1.0}, 0.001);
    dual_arm::addContactGrippers(spec.get());
    auto m = dual_arm::compileSpec(spec.get(), path);
    auto d = dual_arm::MjDataPtr(mj_makeData(m.get()));
    if (!d) throw std::runtime_error("mj_makeData failed");
    // 唯一允许的 equality 是每只手的两指联动（joint 型）；物体与手之间不能有 weld。
    for (int e = 0; e < m->neq; ++e) {
      if (m->eq_type[e] != mjEQ_JOINT) throw std::runtime_error("contact scene must have no weld equality");
    }
    // 与 SimEnv 的 contact_grasp plant 使用同一摩擦模型：椭圆锥 + noslip，消除软接触蠕滑。
    if (cfg.contact_elliptic_cone) m->opt.cone = mjCONE_ELLIPTIC;
    m->opt.noslip_iterations = cfg.contact_noslip_iterations;

    std::array<Arm, 2> arms;
    const std::array<std::string, 2> names = {"left", "right"};
    for (int a = 0; a < 2; ++a) {
      Arm& arm = arms[a];
      arm.initial_q = cfg.scene.q_init[a];
      arm.site = id(m.get(), mjOBJ_SITE, names[a] + "_ee_site");
      for (int j = 0; j < 7; ++j) {
        const std::string joint = names[a] + "_joint" + std::to_string(j + 1);
        const int jid = id(m.get(), mjOBJ_JOINT, joint);
        arm.q[j] = m->jnt_qposadr[jid];
        arm.v[j] = m->jnt_dofadr[jid];
        arm.act[j] = id(m.get(), mjOBJ_ACTUATOR, names[a] + "_actuator" + std::to_string(j + 1));
        d->qpos[arm.q[j]] = arm.initial_q[j];
      }
      for (int f = 0; f < 2; ++f) {
        const std::string base = names[a] + "_" + (f == 0 ? "left" : "right") + "_finger";
        const int jid = id(m.get(), mjOBJ_JOINT, base + "_slide");
        arm.finger_q[f] = m->jnt_qposadr[jid];
        arm.finger_act[f] = id(m.get(), mjOBJ_ACTUATOR, base + "_position");
        arm.finger_body[f] = id(m.get(), mjOBJ_BODY, base);
        d->qpos[arm.finger_q[f]] = 0.04;
        d->ctrl[arm.finger_act[f]] = 0.04;
      }
    }
    const int rod = id(m.get(), mjOBJ_BODY, "rod");
    const int rod_geom = id(m.get(), mjOBJ_GEOM, "rod_geom");
    const int table_geom = id(m.get(), mjOBJ_GEOM, "table_top");
    const int rod_joint = id(m.get(), mjOBJ_JOINT, "rod_joint");
    const int rod_q = m->jnt_qposadr[rod_joint];
    d->qpos[rod_q + 0] = 0.0;
    d->qpos[rod_q + 1] = -0.45;
    d->qpos[rod_q + 2] = 0.781;
    d->qpos[rod_q + 3] = 1.0;
    d->qpos[rod_q + 4] = d->qpos[rod_q + 5] = d->qpos[rod_q + 6] = 0.0;

    // 接触摩擦只增强本独立场景的手指碰撞几何，不修改旧场景。
    for (int g = 0; g < m->ngeom; ++g) {
      for (const Arm& arm : arms) {
        if (m->geom_bodyid[g] == arm.finger_body[0] || m->geom_bodyid[g] == arm.finger_body[1]) {
          m->geom_friction[3 * g] = 1.5;
        }
      }
    }
    mj_forward(m.get(), d.get());
    const double initial_z = d->xpos[3 * rod + 2];

    // 两臂各自的初始线速度 Jacobian 求阻尼最小范数竖直位移；末端姿态靠关节 PD 保持近似。
    std::array<double, 3> initial_tcp_z{};
    for (int a = 0; a < 2; ++a) {
      Arm& arm = arms[a];
      initial_tcp_z[a] = d->site_xpos[3 * arm.site + 2];
      Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> jac(3, m->nv);
      Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> jac_r(3, m->nv);
      mj_jacSite(m.get(), d.get(), jac.data(), jac_r.data(), arm.site);
      Eigen::Matrix<double, 3, 7> J;
      for (int j = 0; j < 7; ++j) J.col(j) = jac.col(arm.v[j]);
      arm.lift_dq = J.transpose() *
          (J * J.transpose() + 0.0025 * Eigen::Matrix3d::Identity()).inverse() * Eigen::Vector3d::UnitZ();
    }

#ifdef DUAL_ARM_HAS_VIEWER
    std::unique_ptr<dual_arm::Viewer> viewer;
    if (!headless) {
      dual_arm::Viewer::Options options;
      options.camera = camera;
      options.title = "Dual-arm contact grasp (no weld)";
      viewer = std::make_unique<dual_arm::Viewer>(m.get(), options);
    }
#else
    if (!headless) throw std::runtime_error("viewer not built; use --headless");
    (void)camera;
#endif
    std::cout << "[contact_grasp] neq=" << m->neq << " (finger couplings only), finger joints=4, object freejoint=1\n";
    std::cout << "[contact_grasp] phases: close 0.2-0.9 s, lift 1.4-3.4 s, hold until 4.0 s\n";
    double max_contact[2] = {0.0, 0.0};
    double final_contact[2] = {0.0, 0.0};
    double finger_contact[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    double final_table = 0.0;
    for (int step = 0; step < 4000; ++step) {
      const double t = d->time;
      mj_step1(m.get(), d.get());
      const double grip_target = 0.04 - 0.020 * smooth((t - 0.2) / 0.7);
      const double lift = 0.10 * smooth((t - 1.4) / 2.0);
      for (Arm& arm : arms) {
        for (int f = 0; f < 2; ++f) d->ctrl[arm.finger_act[f]] = grip_target;
        for (int j = 0; j < 7; ++j) {
          const double reference = arm.initial_q[j] + lift * arm.lift_dq[j];
          const double tau = d->qfrc_bias[arm.v[j]] + 110.0 * (reference - d->qpos[arm.q[j]])
                             - 15.0 * d->qvel[arm.v[j]];
          const int actuator = arm.act[j];
          d->ctrl[actuator] = std::clamp(tau, m->actuator_ctrlrange[2 * actuator],
                                        m->actuator_ctrlrange[2 * actuator + 1]);
        }
      }
      mj_step2(m.get(), d.get());
      final_contact[0] = final_contact[1] = final_table = 0.0;
      for (auto& arm_forces : finger_contact) arm_forces[0] = arm_forces[1] = 0.0;
      for (int c = 0; c < d->ncon; ++c) {
        const mjContact& contact = d->contact[c];
        if (contact.efc_address < 0) continue;
        const int g1 = contact.geom[0], g2 = contact.geom[1];
        const int b1 = m->geom_bodyid[g1], b2 = m->geom_bodyid[g2];
        double wrench[6];
        mj_contactForce(m.get(), d.get(), c, wrench);
        if (g1 == rod_geom || g2 == rod_geom) {
          const int other_body = g1 == rod_geom ? b2 : b1;
          for (int a = 0; a < 2; ++a) {
            for (int f = 0; f < 2; ++f) {
              if (other_body == arms[a].finger_body[f]) {
                finger_contact[a][f] += std::abs(wrench[0]);
                final_contact[a] += std::abs(wrench[0]);
              }
            }
            max_contact[a] = std::max(max_contact[a], final_contact[a]);
          }
          if (g1 == table_geom || g2 == table_geom) final_table += std::abs(wrench[0]);
        }
      }
#ifdef DUAL_ARM_HAS_VIEWER
      if (viewer && step % 16 == 0) {
        char values[256];
        std::snprintf(values, sizeof(values), "t %.2f | rod dz %.1f mm | finger L %.1f N R %.1f N | table %.1f N",
                      d->time, 1000.0 * (d->xpos[3 * rod + 2] - initial_z),
                      final_contact[0], final_contact[1], final_table);
        viewer->render(d.get(), "True contact grasp: close / lift / hold", values);
        if (viewer->shouldClose()) break;
        while (viewer->paused() && !viewer->shouldClose()) viewer->render(d.get(), "Paused", values);
      }
#endif
    }
    const double rise = d->xpos[3 * rod + 2] - initial_z;
    std::cout << "[contact_grasp] final rise=" << 1000.0 * rise << " mm, "
              << "finger normal L/R=" << final_contact[0] << "/" << final_contact[1]
              << " N, table=" << final_table << " N\n";
    std::cout << "[contact_grasp] peak finger normal L/R=" << max_contact[0] << "/"
              << max_contact[1] << " N\n";
    std::cout << "[contact_grasp] four finger normal forces=" << finger_contact[0][0] << "/"
              << finger_contact[0][1] << "/" << finger_contact[1][0] << "/"
              << finger_contact[1][1] << " N\n";
    std::cout << "[contact_grasp] TCP rise L/R="
              << 1000.0 * (d->site_xpos[3 * arms[0].site + 2] - initial_tcp_z[0]) << "/"
              << 1000.0 * (d->site_xpos[3 * arms[1].site + 2] - initial_tcp_z[1])
              << " mm; finger openings=" << d->qpos[arms[0].finger_q[0]] << "/"
              << d->qpos[arms[0].finger_q[1]] << "/" << d->qpos[arms[1].finger_q[0]]
              << "/" << d->qpos[arms[1].finger_q[1]] << " m\n";
    if (check && !(rise > 0.04 && final_table < 0.2
                   && finger_contact[0][0] > 0.2 && finger_contact[0][1] > 0.2
                   && finger_contact[1][0] > 0.2 && finger_contact[1][1] > 0.2)) {
      std::cerr << "[contact_grasp] FAIL: object not stably lifted by both grippers\n";
      return 2;
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[contact_grasp] error: " << e.what() << '\n';
    return 1;
  }
}
