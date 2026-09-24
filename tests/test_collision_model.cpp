// CollisionModel：
//  - 有符号距离 / 最近点 / 法向与 MuJoCo 的 mj_geomDistance 一致，d = nᵀ(p2 − p1)；
//  - ∂d/∂q = nᵀ(J_p2 − J_p1) 与中心差分一致（有限差分检验）；
//  - 被抓物体跟着手动：物体上的点对 q 的雅可比非零。
#include "dual_arm/collision_model.hpp"
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace dual_arm;

namespace {

std::array<Pose, kNumArms> nominalBases(const SimConfig& cfg) {
  return {nominalBasePose(cfg, Arm::Left), nominalBasePose(cfg, Arm::Right)};
}

/// 在 q 下查询并返回 pair p 的距离（不在 margin 内时返回 NaN）
double pairDistance(CollisionModel& cm, const Vector14d& q, int pair) {
  for (const DistanceInfo& di : cm.query(q)) {
    if (di.pair == pair) return di.distance;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

class CollisionModelTest : public ::testing::TestWithParam<std::string> {};

TEST_P(CollisionModelTest, DistancesMatchMujocoGeomDistance) {
  const SimConfig cfg = test::testConfig({}, GetParam());
  CollisionModel cm(cfg.scene, nominalBases(cfg), cfg.collision);
  Vector14d q;
  q << cfg.scene.q_init[0], cfg.scene.q_init[1];
  const auto& out = cm.query(q);
  ASSERT_FALSE(out.empty());
  for (const DistanceInfo& di : out) {
    EXPECT_NEAR(di.normal.norm(), 1.0, 1e-9);
    EXPECT_NEAR(di.normal.dot(di.point2 - di.point1), di.distance, 1e-9);
    mjtNum fromto[6];
    const double d_ref = mj_geomDistance(cm.model(), cm.data(), di.geom1, di.geom2, cm.margin(), fromto);
    EXPECT_NEAR(di.distance, d_ref, 1e-5) << cm.pairName(di.pair);
    if (di.distance > 1e-3) {  // 分离时最近点唯一
      EXPECT_LT((di.point1 - Vector3d(fromto[0], fromto[1], fromto[2])).norm(), 2e-3) << cm.pairName(di.pair);
      EXPECT_LT((di.point2 - Vector3d(fromto[3], fromto[4], fromto[5])).norm(), 2e-3) << cm.pairName(di.pair);
    }
  }
}

TEST_P(CollisionModelTest, GradientMatchesFiniteDifference) {
  // 在 q_init 与 q_init 附近的随机构型上，对每个 margin 内的 pair 取随机方向 v，比较方向导数
  //   解析：g = (∂d/∂q)·v        数值：前向 / 后向差分 f₊ = [d(q + εv) − d(q)]/ε，f₋ = [d(q) − d(q − εv)]/ε
  // d(q) 是凸体距离，在“最近特征切换”处（面—面平行、棱—棱等，初始构型中很常见）只是分段光滑：
  //   光滑点（f₊ ≈ f₋）：要求 g ≈ ½(f₊ + f₋)；
  //   折点（f₊ ≠ f₋）：MuJoCo 给出的是其中一侧最近特征的梯度，要求 g 落在 [min(f₊,f₋), max(f₊,f₋)] 内（次梯度）。
  const SimConfig cfg = test::testConfig({}, GetParam());
  CollisionModel cm(cfg.scene, nominalBases(cfg), cfg.collision);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-0.25, 0.25);
  const double eps = 1e-5;  // GJK 容差 1e-10（CollisionModel 中设置）→ 差分噪声 ~1e-5
  int checked = 0, kinks = 0, refined_points = 0;
  double worst = 0.0;
  for (int trial = 0; trial < 8; ++trial) {
    Vector14d q;
    q << cfg.scene.q_init[0], cfg.scene.q_init[1];
    if (trial > 0) {
      for (int k = 0; k < 14; ++k) q[k] += u(rng);
    }
    const std::vector<DistanceInfo> out = cm.query(q);  // 拷贝：之后的 query 会覆盖缓冲区
    for (const DistanceInfo& di : out) {
      if (di.distance < 2e-3 || di.distance > cm.margin() - 0.01) continue;  // 离 margin 边界太近的不测
      if (di.jacobian.norm() < 1e-6) continue;                               // 两个 geom 都不随 q 动
      Vector14d v;
      for (int k = 0; k < 14; ++k) v[k] = u(rng) * 4.0;
      const double d0 = di.distance;
      const double dp = pairDistance(cm, q + eps * v, di.pair);
      const double dm = pairDistance(cm, q - eps * v, di.pair);
      ASSERT_TRUE(std::isfinite(dp) && std::isfinite(dm));
      double fwd = (dp - d0) / eps, bwd = (d0 - dm) / eps;
      const double an = di.jacobian.dot(v);
      double tol = 1e-4 + 1e-3 * std::abs(0.5 * (fwd + bwd));
      bool refined = false;
      if (std::abs(fwd - bwd) > tol && (an < std::min(fwd, bwd) - tol || an > std::max(fwd, bwd) + tol)) {
        // 小尺寸凸体（指尖垫块 3 mm）在 ε 邻域内可能连续发生两次最近特征切换，此时 ±ε 的单侧差分
        // 已跨过第二个折点，不再是 q 处的单侧导数；用 ε/10 重新求单侧差分再判定。紧邻折点处
        // GJK/EPA 最近点（网格 vs 小盒）的精度约 1%，这类点放宽到 2% 相对误差（ε/10 时差分噪声也更大）。
        const double e2 = 0.1 * eps;
        fwd = (pairDistance(cm, q + e2 * v, di.pair) - d0) / e2;
        bwd = (d0 - pairDistance(cm, q - e2 * v, di.pair)) / e2;
        tol = 1e-3 + 2e-2 * std::abs(0.5 * (fwd + bwd));
        refined = true;
        ++refined_points;
      }
      const double fd = 0.5 * (fwd + bwd);
      if (std::abs(fwd - bwd) > tol) {
        ++kinks;
        EXPECT_GT(an, std::min(fwd, bwd) - tol) << GetParam() << " kink " << cm.pairName(di.pair);
        EXPECT_LT(an, std::max(fwd, bwd) + tol) << GetParam() << " kink " << cm.pairName(di.pair) << ": analytic " << an
                                                << " vs one-sided " << fwd << " / " << bwd;
        continue;
      }
      const double err = std::abs(fd - an);
      ++checked;
      worst = std::max(worst, err / (1e-3 + std::abs(fd)));
      // 光滑点的容差：GJK / EPA 的有限精度 + 凸多面体的特征切换就在 ε 邻域附近时，差分有 ~1e-4 的误差
      EXPECT_LT(err, refined ? tol : 5e-4 + 5e-3 * std::abs(fd)) << GetParam() << " " << cm.pairName(di.pair) << " d = " << di.distance << ": analytic " << an
                          << " vs FD " << fd;
    }
  }
  EXPECT_GT(checked, 50);
  EXPECT_LT(kinks, checked / 2);
  EXPECT_LE(refined_points, 5);  // 需要 ε/10 复核的点只能是个例
  std::printf("[collision FD] %s: %d smooth points (worst rel. err %.2e), %d kinks (subgradient check), %d refined\n",
              GetParam().c_str(), checked, worst, kinks, refined_points);
}

INSTANTIATE_TEST_SUITE_P(Scenes, CollisionModelTest, ::testing::Values("lift", "slot", "assembly"),
                         [](const ::testing::TestParamInfo<std::string>& p) { return p.param; });

TEST(CollisionModel, GraspedObjectMovesWithHand) {
  // slot：板被左手“拿着”（按左手计算），板—槽座的距离对左臂关节的导数非零、对右臂为零
  const SimConfig cfg = test::testConfig({}, "slot");
  CollisionModel cm(cfg.scene, nominalBases(cfg), cfg.collision);
  int group = -1;
  for (int g = 0; g < cm.numGroups(); ++g) {
    if (cm.groupName(g) == "object~slot_frame") group = g;
  }
  ASSERT_GE(group, 0);
  bool found = false;
  for (const DistanceInfo& di : cm.query(cfg.scene.q_init[0], cfg.scene.q_init[1])) {
    if (di.group != group) continue;
    found = true;
    EXPECT_GT(di.jacobian.head<7>().norm(), 1e-3);
    EXPECT_LT(di.jacobian.tail<7>().norm(), 1e-12);
  }
  EXPECT_TRUE(found);
}

TEST(CollisionModel, TooManyPairsIsAnError) {
  SimConfig cfg = test::testConfig({}, "lift");
  cfg.collision.max_pairs = 10;
  EXPECT_THROW(CollisionModel(cfg.scene, nominalBases(cfg), cfg.collision), std::runtime_error);
}
