#include "dual_arm/mujoco_utils.hpp"

#include <stdexcept>
#include <string>

namespace dual_arm {
namespace {

int requireId(const mjModel* m, mjtObj type, const std::string& name) {
  const int id = mj_name2id(m, type, name.c_str());
  if (id < 0) throw std::runtime_error("MuJoCo model has no element named '" + name + "'");
  return id;
}

bool collides(const mjModel* m, int g) { return m->geom_contype[g] != 0 || m->geom_conaffinity[g] != 0; }

/// body 的 free joint 地址（没有则抛异常）
void freeJointOf(const mjModel* m, int body, int* qpos_adr, int* dof_adr) {
  const int j = m->body_jntadr[body];
  if (j < 0 || m->body_jntnum[body] != 1 || m->jnt_type[j] != mjJNT_FREE) {
    throw std::runtime_error(std::string("body '") + mj_id2name(m, mjOBJ_BODY, body) +
                             "' must have exactly one free joint");
  }
  *qpos_adr = m->jnt_qposadr[j];
  *dof_adr = m->jnt_dofadr[j];
}

}  // namespace

Pose poseFromMj(const mjtNum* pos, const mjtNum* quat_wxyz) {
  Pose T;
  T.p = Vector3d(pos[0], pos[1], pos[2]);
  T.q = Quaterniond(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]).normalized();
  return T;
}

Pose poseFromMjMat(const mjtNum* pos, const mjtNum* xmat) {
  Pose T;
  T.p = Vector3d(pos[0], pos[1], pos[2]);
  // MuJoCo 的 xmat 按行主序存储
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(xmat);
  T.q = Quaterniond(Matrix3d(R)).normalized();
  return T;
}

MjSpecPtr loadSceneSpec(const std::string& xml_path, const std::array<Pose, kNumArms>& base_poses,
                        const std::array<double, kNumArms>& finger_openings, double timestep) {
  char err[1024] = "";
  MjSpecPtr spec(mj_parseXML(xml_path.c_str(), nullptr, err, sizeof(err)));
  if (!spec) throw std::runtime_error("failed to parse '" + xml_path + "': " + err);

  for (Arm a : kArms) {
    const std::string pre(armName(a));
    mjsBody* body = mjs_findBody(spec.get(), (pre + "_base").c_str());
    if (!body) throw std::runtime_error("scene has no body '" + pre + "_base'");
    const Pose& T = base_poses[armIndex(a)];
    for (int k = 0; k < 3; ++k) body->pos[k] = T.p[k];
    body->quat[0] = T.q.w();
    body->quat[1] = T.q.x();
    body->quat[2] = T.q.y();
    body->quat[3] = T.q.z();
    body->alt.type = mjORIENTATION_QUAT;  // 让 quat 生效（XML 中若用了 euler 等替代表示会覆盖 quat）

    const double op = finger_openings[armIndex(a)];
    if (op >= 0.0) {
      // 手指固定（无关节）：左指沿 hand +y、右指（绕 z 转 180°）沿 −y 放在开度处
      mjsBody* lf = mjs_findBody(spec.get(), (pre + "_left_finger").c_str());
      mjsBody* rf = mjs_findBody(spec.get(), (pre + "_right_finger").c_str());
      if (!lf || !rf) throw std::runtime_error("scene has no fingers for arm " + pre);
      lf->pos[1] = op;
      rf->pos[1] = -op;
    }
  }
  spec->option.timestep = timestep;
  return spec;
}

MjModelPtr compileSpec(mjSpec* spec, const std::string& what) {
  mjModel* m = mj_compile(spec, nullptr);
  if (!m) throw std::runtime_error("failed to compile '" + what + "': " + mjs_getError(spec));
  return MjModelPtr(m);
}

MjModelPtr loadSceneModel(const std::string& xml_path, const std::array<Pose, kNumArms>& base_poses,
                          const std::array<double, kNumArms>& finger_openings, double timestep,
                          const std::function<void(mjSpec*)>& spec_edit) {
  MjSpecPtr spec = loadSceneSpec(xml_path, base_poses, finger_openings, timestep);
  if (spec_edit) spec_edit(spec.get());
  return compileSpec(spec.get(), xml_path);
}

SceneIndices findSceneIndices(const mjModel* m, const SceneSpec& scene) {
  SceneIndices idx;
  for (Arm a : kArms) {
    const std::string pre = std::string(armName(a)) + "_";
    ArmIndices& ai = idx.arm[armIndex(a)];
    for (int j = 0; j < kArmDof; ++j) {
      ai.joint_id[j] = requireId(m, mjOBJ_JOINT, pre + "joint" + std::to_string(j + 1));
      ai.actuator_id[j] = requireId(m, mjOBJ_ACTUATOR, pre + "actuator" + std::to_string(j + 1));
      const int jid = ai.joint_id[j];
      if (m->jnt_type[jid] != mjJNT_HINGE) {
        throw std::runtime_error(pre + "joint" + std::to_string(j + 1) + " is not a hinge joint");
      }
      if (j == 0) {
        ai.qpos_adr = m->jnt_qposadr[jid];
        ai.dof_adr = m->jnt_dofadr[jid];
      } else if (m->jnt_qposadr[jid] != ai.qpos_adr + j || m->jnt_dofadr[jid] != ai.dof_adr + j) {
        throw std::runtime_error("joints of arm '" + pre + "' are not contiguous in qpos/qvel");
      }
      const int act = ai.actuator_id[j];
      // 必须是直接作用在关节上的力矩电机：gain=1、无 bias、传动比 1
      if (m->actuator_trntype[act] != mjTRN_JOINT || m->actuator_trnid[2 * act] != jid ||
          m->actuator_biastype[act] != mjBIAS_NONE || m->actuator_gainprm[mjNGAIN * act] != 1.0 ||
          m->actuator_gear[6 * act] != 1.0) {
        throw std::runtime_error("actuator '" + pre + "actuator" + std::to_string(j + 1) +
                                 "' is not a unit-gain torque motor on its joint");
      }
    }
    ai.base_body = requireId(m, mjOBJ_BODY, std::string(armName(a)) + "_base");
    ai.hand_body = requireId(m, mjOBJ_BODY, pre + "hand");
    ai.ee_site = requireId(m, mjOBJ_SITE, pre + "ee_site");
    ai.ft_site = requireId(m, mjOBJ_SITE, pre + "ft_site");
    ai.ft_force_adr = m->sensor_adr[requireId(m, mjOBJ_SENSOR, pre + "ft_force")];
    ai.ft_torque_adr = m->sensor_adr[requireId(m, mjOBJ_SENSOR, pre + "ft_torque")];

    const GraspSpec& g = scene.graspOf(a);
    ai.grasp_site = requireId(m, mjOBJ_SITE, g.site);
    ai.grasp_body = m->site_bodyid[ai.grasp_site];
    freeJointOf(m, ai.grasp_body, &ai.grasp_qpos_adr, &ai.grasp_dof_adr);
    ai.weld_eq = requireId(m, mjOBJ_EQUALITY, g.weld);
    if (m->eq_type[ai.weld_eq] != mjEQ_WELD || m->eq_objtype[ai.weld_eq] != mjOBJ_BODY ||
        m->eq_obj1id[ai.weld_eq] != ai.hand_body || m->eq_obj2id[ai.weld_eq] != ai.grasp_body) {
      throw std::runtime_error("grasp weld '" + g.weld + "' must be a body weld with body1 = " + pre +
                               "hand and body2 = the body of site '" + g.site + "'");
    }
  }

  idx.object_body = requireId(m, mjOBJ_BODY, scene.object.body);
  freeJointOf(m, idx.object_body, &idx.object_qpos_adr, &idx.object_dof_adr);

  for (const auto& c : scene.constraints) {
    const int b1 = requireId(m, mjOBJ_BODY, c.body1);
    const int b2 = requireId(m, mjOBJ_BODY, c.body2);
    if (c.type == ConstraintType::Rigid) {
      const int eq = requireId(m, mjOBJ_EQUALITY, c.name);
      if (m->eq_type[eq] != mjEQ_WELD || m->eq_objtype[eq] != mjOBJ_BODY || m->eq_obj1id[eq] != b1 ||
          m->eq_obj2id[eq] != b2) {
        throw std::runtime_error("rigid constraint '" + c.name + "' must be a body weld " + c.body1 + " -> " +
                                 c.body2);
      }
      idx.rigid_welds.push_back(eq);
    } else {
      ScrewIndices& s = idx.screw;
      if (s.valid) throw std::runtime_error("only one screw constraint is supported");
      s.valid = true;
      s.body1 = b1;
      s.body2 = b2;
      s.hinge_joint = requireId(m, mjOBJ_JOINT, c.hinge);
      s.slide_joint = requireId(m, mjOBJ_JOINT, c.slide);
      if (m->jnt_type[s.hinge_joint] != mjJNT_HINGE || m->jnt_type[s.slide_joint] != mjJNT_SLIDE ||
          m->jnt_bodyid[s.hinge_joint] != b2 || m->jnt_bodyid[s.slide_joint] != b2 || m->body_parentid[b2] != b1) {
        throw std::runtime_error("screw constraint '" + c.name + "': hinge/slide must be joints of body2 (" +
                                 c.body2 + "), whose parent is body1 (" + c.body1 + ")");
      }
      s.hinge_qpos = m->jnt_qposadr[s.hinge_joint];
      s.hinge_dof = m->jnt_dofadr[s.hinge_joint];
      s.slide_qpos = m->jnt_qposadr[s.slide_joint];
      s.slide_dof = m->jnt_dofadr[s.slide_joint];
      if (!c.coupling.empty()) requireId(m, mjOBJ_EQUALITY, c.coupling);
    }
  }
  return idx;
}

void completeSceneSpec(SceneSpec& scene, const mjModel* m) {
  ObjectSpec& o = scene.object;
  const int b = requireId(m, mjOBJ_BODY, o.body);
  o.mass = m->body_mass[b];
  o.com = Vector3d(m->body_ipos[3 * b], m->body_ipos[3 * b + 1], m->body_ipos[3 * b + 2]);
  const Quaterniond qi(m->body_iquat[4 * b], m->body_iquat[4 * b + 1], m->body_iquat[4 * b + 2],
                       m->body_iquat[4 * b + 3]);
  const Matrix3d Ri = qi.normalized().toRotationMatrix();
  const Vector3d I_diag(m->body_inertia[3 * b], m->body_inertia[3 * b + 1], m->body_inertia[3 * b + 2]);
  o.inertia = Ri * I_diag.asDiagonal() * Ri.transpose();
  if (!o.geom.empty()) {
    const int g = requireId(m, mjOBJ_GEOM, o.geom);
    o.half_size = Vector3d(m->geom_size[3 * g], m->geom_size[3 * g + 1], m->geom_size[3 * g + 2]);
  }
  for (Arm a : kArms) {
    GraspSpec& g = scene.grasp[armIndex(a)];
    const int s = requireId(m, mjOBJ_SITE, g.site);
    g.body = mj_id2name(m, mjOBJ_BODY, m->site_bodyid[s]);
    g.site_in_body = poseFromMj(m->site_pos + 3 * s, m->site_quat + 4 * s);
  }
  for (const auto& c : scene.constraints) {
    requireId(m, mjOBJ_BODY, c.body1);
    requireId(m, mjOBJ_BODY, c.body2);
  }
  for (const auto& p : scene.obstacles) {
    if (resolveGeomGroup(m, scene, p.a).empty() || resolveGeomGroup(m, scene, p.b).empty()) {
      throw std::runtime_error("obstacle pair [" + p.a + ", " + p.b + "] resolves to no collision geom");
    }
  }
  for (const auto& c : scene.cameras) requireId(m, mjOBJ_CAMERA, c);
}

std::vector<int> resolveGeomGroup(const mjModel* m, const SceneSpec& scene, const std::string& name) {
  std::vector<int> out;
  auto addBody = [&](int body) {
    for (int g = 0; g < m->ngeom; ++g) {
      if (m->geom_bodyid[g] == body && collides(m, g)) out.push_back(g);
    }
  };
  if (name == "left_arm" || name == "right_arm") {
    const std::string pre = name.substr(0, name.find('_')) + "_";
    for (int b = 0; b < m->nbody; ++b) {
      const char* bn = mj_id2name(m, mjOBJ_BODY, b);
      if (!bn) continue;
      const std::string s(bn);
      if (s.rfind(pre, 0) == 0 && s != pre + "base" && s != pre + "link0") addBody(b);
    }
  } else if (name == "object") {
    addBody(requireId(m, mjOBJ_BODY, scene.object.body));
  } else if (const int b = mj_name2id(m, mjOBJ_BODY, name.c_str()); b >= 0) {
    addBody(b);
  } else {
    const int g = requireId(m, mjOBJ_GEOM, name);
    if (collides(m, g)) out.push_back(g);
  }
  return out;
}

}  // namespace dual_arm
