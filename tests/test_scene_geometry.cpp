// 场景几何检查（四个场景，初始构型 q_init、无标定误差）：
//  - TCP（ee_site）与抓取点 site 重合（位置与姿态），TCP 在法兰前方 0.1034 m；
//  - 夹爪指面贴住被抓 body 但不穿透：手指碰撞体与被抓 body 的间隙 ≈ 1 mm；
//  - 两臂的碰撞体与场景中其它碰撞体（含另一条臂、台面、障碍、物体 / 工具）都不穿透；
//  - 初始时刻没有任何接触；基座装在台面上；
//  - 夹爪质量保留（hand 0.73 kg + 两指 0.015 kg），手指没有关节；SceneSpec 中的相机都存在；
//  - 布局只来自 config：模型中的基座位姿 = config.layout。
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

using namespace dual_arm;

namespace {

bool startsWith(const char* s, const char* prefix) { return s && std::string(s).rfind(prefix, 0) == 0; }
bool isArmBody(const mjModel* m, int body) {
  const char* n = mj_id2name(m, mjOBJ_BODY, body);
  return startsWith(n, "left_") || startsWith(n, "right_");
}
bool collides(const mjModel* m, int g) { return m->geom_contype[g] != 0 || m->geom_conaffinity[g] != 0; }

}  // namespace

class SceneGeometry : public ::testing::TestWithParam<std::string> {
 protected:
  void SetUp() override {
    cfg_ = test::testConfig({}, GetParam());
    env_ = std::make_unique<SimEnv>(cfg_);
    m_ = env_->model();
    d_ = env_->data();
  }
  SimConfig cfg_;
  std::unique_ptr<SimEnv> env_;
  const mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
};

TEST_P(SceneGeometry, TcpCoincidesWithGraspSite) {
  for (Arm a : kArms) {
    const auto& ai = env_->indices()[a];
    const Pose tcp = poseFromMjMat(d_->site_xpos + 3 * ai.ee_site, d_->site_xmat + 9 * ai.ee_site);
    const Pose g = poseFromMjMat(d_->site_xpos + 3 * ai.grasp_site, d_->site_xmat + 9 * ai.grasp_site);
    EXPECT_LT((tcp.p - g.p).norm(), 1e-5) << armName(a);
    EXPECT_LT(rotationError(tcp.R(), g.R()).norm(), 1e-4) << armName(a);
    const Pose hand = poseFromMj(d_->xpos + 3 * ai.hand_body, d_->xquat + 4 * ai.hand_body);
    EXPECT_LT((hand.inverse().transformPoint(tcp.p) - Vector3d(0, 0, 0.1034)).norm(), 1e-9);
  }
}

TEST_P(SceneGeometry, FingersTouchGraspedBodyWithoutPenetration) {
  for (Arm a : kArms) {
    const int grasped = env_->indices()[a].grasp_body;
    for (const char* finger : {"_left_finger", "_right_finger"}) {
      const int body = mj_name2id(m_, mjOBJ_BODY, (std::string(armName(a)) + finger).c_str());
      ASSERT_GE(body, 0);
      double min_pad = std::numeric_limits<double>::infinity();
      for (int g = 0; g < m_->ngeom; ++g) {
        if (m_->geom_bodyid[g] != body || !collides(m_, g)) continue;
        for (int h = 0; h < m_->ngeom; ++h) {
          if (m_->geom_bodyid[h] != grasped || !collides(m_, h)) continue;
          const double dist = mj_geomDistance(m_, d_, g, h, 0.1, nullptr);
          EXPECT_GE(dist, 0.0) << armName(a) << finger << " penetrates the grasped body";
          min_pad = std::min(min_pad, dist);
        }
      }
      // finger_opening = 被夹宽度/2 + 1 mm：平面被夹面（杆、板、工件）间隙 = 1 mm；
      // 圆柱形工具杆（r = 6 mm）与偏离轴线约 2.5 mm 的指垫碰撞盒之间为 √(7² + 2.5²) − 6 ≈ 1.43 mm
      EXPECT_GT(min_pad, 0.75e-3) << GetParam() << " " << armName(a) << finger;
      EXPECT_LT(min_pad, 1.5e-3) << GetParam() << " " << armName(a) << finger;
    }
  }
}

TEST_P(SceneGeometry, ArmsDoNotPenetrateAnything) {
  for (int g = 0; g < m_->ngeom; ++g) {
    if (!collides(m_, g) || !isArmBody(m_, m_->geom_bodyid[g])) continue;
    for (int h = 0; h < m_->ngeom; ++h) {
      if (h == g || !collides(m_, h)) continue;
      const int bg = m_->geom_bodyid[g], bh = m_->geom_bodyid[h];
      if (isArmBody(m_, bh)) {
        // 同一条臂内部的相邻连杆本来就重叠（由 parent 过滤），只检查左右臂之间
        const std::string ng = mj_id2name(m_, mjOBJ_BODY, bg), nh = mj_id2name(m_, mjOBJ_BODY, bh);
        if (ng.substr(0, ng.find('_')) == nh.substr(0, nh.find('_')) || h < g) continue;
      }
      if (m_->body_weldid[bg] == 0 && m_->body_weldid[bh] == 0) continue;  // link0 与台面：都是静止的
      const double dist = mj_geomDistance(m_, d_, g, h, 0.05, nullptr);
      EXPECT_GE(dist, 0.0) << GetParam() << ": " << mj_id2name(m_, mjOBJ_BODY, bg) << " geom " << g << " vs "
                           << mj_id2name(m_, mjOBJ_BODY, bh) << " geom " << h;
    }
  }
}

TEST_P(SceneGeometry, NoContactsAtStartAndBasesOnTable) {
  EXPECT_EQ(d_->ncon, 0) << GetParam();
  const int top = mj_name2id(m_, mjOBJ_GEOM, "table_top");
  ASSERT_GE(top, 0);
  const double table_top_z = d_->geom_xpos[3 * top + 2] + m_->geom_size[3 * top + 2];
  for (Arm a : kArms) {
    EXPECT_NEAR(env_->basePose(a).p.z(), table_top_z, 1e-3);
    // 基座位姿只来自 config.layout
    EXPECT_LT((env_->basePose(a).p - cfg_.base[armIndex(a)].pos).norm(), 1e-12);
    EXPECT_LT(rotationError(env_->basePose(a).R(), quatFromRpyDeg(cfg_.base[armIndex(a)].rpy_deg).toRotationMatrix())
                  .norm(),
              1e-12);
  }
}

TEST_P(SceneGeometry, HandMassKeptAndFingersFixed) {
  for (Arm a : kArms) {
    const int hand = env_->indices()[a].hand_body;
    EXPECT_NEAR(m_->body_mass[hand], 0.73, 1e-9);
    EXPECT_NEAR(m_->body_subtreemass[hand], 0.76, 1e-9);
    for (const char* finger : {"_left_finger", "_right_finger"}) {
      const int body = mj_name2id(m_, mjOBJ_BODY, (std::string(armName(a)) + finger).c_str());
      EXPECT_EQ(m_->body_jntnum[body], 0) << "fingers must be fixed";
    }
  }
  EXPECT_EQ(m_->nu, 2 * kArmDof);
}

TEST_P(SceneGeometry, SceneSpecIsConsistentWithModel) {
  const SceneSpec& sc = env_->scene();
  for (const auto& c : sc.cameras) EXPECT_GE(mj_name2id(m_, mjOBJ_CAMERA, c.c_str()), 0) << c;
  EXPECT_GT(sc.object.mass, 0.0);
  EXPECT_GE(sc.waypoints.size(), 1u);
  // 第一个航点 = 物体初始位姿
  EXPECT_LT((env_->state().object.pose.p - sc.waypoints.front().pose.p).norm(), 1e-9);
  const int expected_rel = GetParam() == "assembly" ? 1 : 0;
  EXPECT_EQ(sc.relativeDofBetweenHands(), expected_rel);
}

INSTANTIATE_TEST_SUITE_P(AllScenes, SceneGeometry, ::testing::ValuesIn(test::allScenes()),
                         [](const ::testing::TestParamInfo<std::string>& p) { return p.param; });
