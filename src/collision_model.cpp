#include "dual_arm/collision_model.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace dual_arm {

CollisionModel::CollisionModel(const SceneSpec& scene, const std::array<Pose, kNumArms>& base_poses,
                               const CollisionConfig& cfg, double timestep)
    : scene_(scene), margin_(cfg.margin) {
  const std::array<double, kNumArms> fingers{scene.grasp[0].finger_opening, scene.grasp[1].finger_opening};
  MjSpecPtr spec = loadSceneSpec(scene.model_path, base_poses, fingers, timestep);

  // 1) 给所有无名 geom 起名字（显式 pair 只能按名字引用）
  int geom_counter = 0;
  for (mjsElement* el = mjs_firstElement(spec.get(), mjOBJ_GEOM); el;
       el = mjs_nextElement(spec.get(), el), ++geom_counter) {
    const char* n = mjs_getString(mjs_getName(el));
    if (!n || !*n) mjs_setName(el, ("__cm_geom_" + std::to_string(geom_counter)).c_str());
  }
  // 2) 先编译一次，按 SceneSpec 的规则把 obstacles 的名字解析成 geom
  MjModelPtr probe = compileSpec(spec.get(), scene.model_path);
  completeSceneSpec(scene_, probe.get());
  std::vector<std::array<std::string, 2>> names;
  for (std::size_t gi = 0; gi < scene_.obstacles.size(); ++gi) {
    const auto& op = scene_.obstacles[gi];
    group_name_.push_back(op.a + "~" + op.b);
    const std::vector<int> ga = resolveGeomGroup(probe.get(), scene_, op.a);
    const std::vector<int> gb = resolveGeomGroup(probe.get(), scene_, op.b);
    for (int a : ga) {
      for (int b : gb) {
        if (a == b) continue;
        const std::string na = mj_id2name(probe.get(), mjOBJ_GEOM, a);
        const std::string nb = mj_id2name(probe.get(), mjOBJ_GEOM, b);
        const bool dup = std::any_of(names.begin(), names.end(), [&](const auto& p) {
          return (p[0] == na && p[1] == nb) || (p[0] == nb && p[1] == na);
        });
        if (dup) continue;
        names.push_back({na, nb});
        pair_group_.push_back(static_cast<int>(gi));
      }
    }
  }
  if (static_cast<int>(names.size()) > cfg.max_pairs) {
    throw std::runtime_error("CollisionModel: " + std::to_string(names.size()) +
                             " geom pairs exceed collision.max_pairs = " + std::to_string(cfg.max_pairs));
  }
  // 3) 参与 pair 的、会动的 box geom（指垫、被抓物体 / 工具）换成同尺寸的 8 顶点凸网格。
  //    MuJoCo 的 box–box 专用碰撞函数（mjc_BoxBox）在分离状态下给出的是“分离轴 + 面裁剪”意义下的接触距离
  //    与接触点，不是欧氏最近点对，其梯度与 d(q) 不一致（实测差几 % 到几十 %）。换成网格后 box–box 变成
  //    mesh–box，走 GJK / EPA（mjc_Convex），返回精确的欧氏距离与最近点。
  //    静止的 box（台面、台阶、槽座……）保持原样：两个都换成网格时，面完全平行的一对网格会让 GJK 退化，
  //    MuJoCo 3.12 的 EPA 在这种情况下会段错误（mjc_penetration → projectOriginPlane）。
  {
    std::vector<std::string> moving;
    for (const auto& p : names) {
      for (const std::string& n : p) {
        const int g = mj_name2id(probe.get(), mjOBJ_GEOM, n.c_str());
        const bool is_static = probe->body_weldid[probe->geom_bodyid[g]] == 0;
        if (!is_static && probe->geom_type[g] == mjGEOM_BOX &&
            std::find(moving.begin(), moving.end(), n) == moving.end()) {
          moving.push_back(n);
        }
      }
    }
    int nb = 0;
    for (const std::string& n : moving) {
      mjsGeom* g = mjs_asGeom(mjs_findElement(spec.get(), mjOBJ_GEOM, n.c_str()));
      const float hx = static_cast<float>(g->size[0]), hy = static_cast<float>(g->size[1]),
                  hz = static_cast<float>(g->size[2]);
      const float vert[24] = {-hx, -hy, -hz, hx, -hy, -hz, -hx, hy, -hz, hx, hy, -hz,
                              -hx, -hy, hz,  hx, -hy, hz,  -hx, hy, hz,  hx, hy, hz};
      mjsMesh* mesh = mjs_addMesh(spec.get(), nullptr);
      const std::string mname = "__cm_boxmesh_" + std::to_string(nb++);
      mjs_setName(mesh->element, mname.c_str());
      mjs_setFloat(mesh->uservert, vert, 24);
      g->type = mjGEOM_MESH;
      mjs_setString(g->meshname, mname.c_str());
    }
  }
  // 4) 加显式 pair（大 margin），关闭其余所有碰撞
  for (const auto& p : names) {
    mjsPair* pr = mjs_addPair(spec.get(), nullptr);
    mjs_setString(pr->geomname1, p[0].c_str());
    mjs_setString(pr->geomname2, p[1].c_str());
    pr->margin = margin_;
    pr->gap = 0.0;
  }
  for (mjsElement* el = mjs_firstElement(spec.get(), mjOBJ_GEOM); el; el = mjs_nextElement(spec.get(), el)) {
    mjsGeom* g = mjs_asGeom(el);
    g->contype = 0;
    g->conaffinity = 0;
  }
  // 凸体距离（GJK / EPA）求解精度：默认的容差与迭代次数下，距离对 q 的有限差分有 ~1e-2 的噪声，
  // 收紧到 1e-10 / 100 次后与解析梯度的差 < 1%（tests/test_collision_model.cpp）。
  spec->option.ccd_tolerance = 1e-10;
  spec->option.ccd_iterations = 100;
  // 距离查询每个 pair 只需要一个 contact（最近点对）：关掉 3.12 默认开启的 multiccd（一对凸体生成多个接触点），
  // 查询耗时降为约 1/4.5
  spec->option.disableflags |= mjDSBL_MULTICCD;
  // 大 margin 下 contact 数量远多于正常仿真：给 arena 足够的空间（不足时 MuJoCo 会丢 contact 并报 CONTACTFULL）
  spec->memory = std::max<mjtSize>(spec->memory, 64 << 20);
  model_ = compileSpec(spec.get(), scene.model_path);
  data_ = MjDataPtr(mj_makeData(model_.get()));
  if (!data_) throw std::runtime_error("mj_makeData failed");
  if (cfg.threads > 1) mju_threadpool(data_.get(), cfg.threads);  // MuJoCo 内部线程池：窄相碰撞并行
  const mjModel* m = model_.get();
  if (m->npair != static_cast<int>(names.size())) throw std::logic_error("CollisionModel: pair count mismatch");

  // 5) 索引
  const SceneIndices idx = findSceneIndices(m, scene_);
  for (Arm a : kArms) {
    const int i = armIndex(a);
    qpos_adr_[i] = idx[a].qpos_adr;
    dof_adr_[i] = idx[a].dof_adr;
    ee_site_[i] = idx[a].ee_site;
    hand_body_[i] = idx[a].hand_body;
  }
  // 编译器会对显式 pair 重新排序（并可能交换 geom1/geom2），所以按名字把编译后的 pair 对应回 obstacles 的组，
  // 并统一方向：geom1 属于 obstacles[group].a
  const std::vector<int> group_of_added = pair_group_;
  pair_group_.assign(m->npair, -1);
  pair_geom_.resize(m->npair);
  pair_lookup_.assign(static_cast<std::size_t>(m->ngeom) * m->ngeom, -1);
  for (int p = 0; p < m->npair; ++p) {
    const int g1 = m->pair_geom1[p], g2 = m->pair_geom2[p];
    const std::string n1 = mj_id2name(m, mjOBJ_GEOM, g1), n2 = mj_id2name(m, mjOBJ_GEOM, g2);
    for (std::size_t k = 0; k < names.size(); ++k) {
      if (names[k][0] == n1 && names[k][1] == n2) {
        pair_geom_[p] = {g1, g2};
      } else if (names[k][0] == n2 && names[k][1] == n1) {
        pair_geom_[p] = {g2, g1};
      } else {
        continue;
      }
      pair_group_[p] = group_of_added[k];
      break;
    }
    if (pair_group_[p] < 0) throw std::logic_error("CollisionModel: cannot match compiled pair " + n1 + "~" + n2);
    pair_lookup_[static_cast<std::size_t>(g1) * m->ngeom + g2] = p;
    pair_lookup_[static_cast<std::size_t>(g2) * m->ngeom + g1] = p;
  }
  // 被抓 body（主物体被两只手抓时只按第一只手——左手——计算）
  for (Arm a : kArms) {
    const int b = idx[a].grasp_body;
    const bool dup = std::any_of(attached_.begin(), attached_.end(), [&](const Attached& x) { return x.body == b; });
    if (dup) continue;
    attached_.push_back({b, idx[a].grasp_qpos_adr, a, scene_.graspOf(a).site_in_body});
  }
  jac_body_.resize(m->ngeom);
  for (int g = 0; g < m->ngeom; ++g) {
    const int b = m->geom_bodyid[g];
    const int root = m->body_rootid[b];  // 被抓 body 及其子树（例如螺钉）都跟着手动
    jac_body_[g] = b;
    for (const Attached& at : attached_) {
      if (root == at.body) jac_body_[g] = hand_body_[armIndex(at.arm)];
    }
  }

  slot_.assign(m->npair, -1);
  out_.reserve(m->npair);
  jac1_.assign(3 * m->nv, 0.0);
  jac2_.assign(3 * m->nv, 0.0);
  mj_resetData(m, data_.get());
  query(scene_.q_init[0], scene_.q_init[1]);
}

const std::vector<DistanceInfo>& CollisionModel::query(const Vector7d& q_left, const Vector7d& q_right) {
  const mjModel* m = model_.get();
  mjData* d = data_.get();
  Eigen::Map<Vector7d>(d->qpos + qpos_adr_[0]) = q_left;
  Eigen::Map<Vector7d>(d->qpos + qpos_adr_[1]) = q_right;
  // 被抓 body 放到手里：T_body = T_tcp · T_grasp⁻¹
  mj_kinematics(m, d);
  for (const Attached& at : attached_) {
    const int s = ee_site_[armIndex(at.arm)];
    const Pose T = poseFromMjMat(d->site_xpos + 3 * s, d->site_xmat + 9 * s) * at.grasp_in_body.inverse();
    mjtNum* qp = d->qpos + at.qpos_adr;
    qp[0] = T.p.x();
    qp[1] = T.p.y();
    qp[2] = T.p.z();
    qp[3] = T.q.w();
    qp[4] = T.q.x();
    qp[5] = T.q.y();
    qp[6] = T.q.z();
  }
  mj_kinematics(m, d);
  mj_comPos(m, d);  // mj_jac 需要 cdof / subtree_com
  mj_collision(m, d);

  // 每个 pair 只保留 d 最小的 contact
  out_.clear();
  for (int c = 0; c < d->ncon; ++c) {
    const mjContact& con = d->contact[c];
    const int g0 = con.geom[0], g1 = con.geom[1];
    if (g0 < 0 || g1 < 0) continue;
    const int p = pair_lookup_[static_cast<std::size_t>(g0) * m->ngeom + g1];
    if (p < 0) continue;
    int s = slot_[p];
    if (s >= 0 && out_[s].distance <= con.dist) continue;
    if (s < 0) {
      s = static_cast<int>(out_.size());
      out_.emplace_back();  // 容量已预留 npair，不会重新分配
      slot_[p] = s;
    }
    DistanceInfo& di = out_[s];
    di.pair = p;
    di.group = pair_group_[p];
    di.distance = con.dist;
    Vector3d n(con.frame[0], con.frame[1], con.frame[2]);
    const Vector3d mid(con.pos[0], con.pos[1], con.pos[2]);
    Vector3d p0 = mid - 0.5 * con.dist * n;  // geom[0] 上的最近点
    Vector3d p1 = mid + 0.5 * con.dist * n;  // geom[1] 上的最近点
    // 统一方向：geom1 = pair 定义中的第一个（属于 obstacles[group].a）
    if (g0 != pair_geom_[p][0]) {
      n = -n;
      std::swap(p0, p1);
    }
    di.geom1 = pair_geom_[p][0];
    di.geom2 = pair_geom_[p][1];
    di.normal = n;
    di.point1 = p0;
    di.point2 = p1;
  }
  for (const DistanceInfo& di : out_) slot_[di.pair] = -1;

  // ∂d/∂q = nᵀ (J_p(body2, p2) − J_p(body1, p1))，只取两臂的 14 列
  using RowMat3 = Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>;
  const int nv = m->nv;
  for (DistanceInfo& di : out_) {
    mj_jac(m, d, jac1_.data(), nullptr, di.point1.data(), jac_body_[di.geom1]);
    mj_jac(m, d, jac2_.data(), nullptr, di.point2.data(), jac_body_[di.geom2]);
    const Eigen::Map<const RowMat3> J1(jac1_.data(), 3, nv), J2(jac2_.data(), 3, nv);
    for (Arm a : kArms) {
      const int i = armIndex(a);
      di.jacobian.segment<7>(7 * i) =
          di.normal.transpose() * (J2.middleCols<kArmDof>(dof_adr_[i]) - J1.middleCols<kArmDof>(dof_adr_[i]));
    }
  }
  return out_;
}

double CollisionModel::groupMinDistance(int group) const {
  double dmin = margin_;
  for (const DistanceInfo& di : out_) {
    if (di.group == group) dmin = std::min(dmin, di.distance);
  }
  return dmin;
}

double CollisionModel::minDistance(int* index) const {
  double dmin = margin_;
  if (index) *index = -1;
  for (std::size_t i = 0; i < out_.size(); ++i) {
    if (out_[i].distance < dmin) {
      dmin = out_[i].distance;
      if (index) *index = static_cast<int>(i);
    }
  }
  return dmin;
}

std::string CollisionModel::pairName(int pair) const {
  const mjModel* m = model_.get();
  const char* a = mj_id2name(m, mjOBJ_GEOM, pair_geom_[pair][0]);
  const char* b = mj_id2name(m, mjOBJ_GEOM, pair_geom_[pair][1]);
  return std::string(a ? a : "?") + "~" + (b ? b : "?");
}

}  // namespace dual_arm
