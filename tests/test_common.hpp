#pragma once
// 测试共用的小工具
#include "dual_arm/config.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/sim_env.hpp"

#include <random>
#include <string>
#include <vector>

namespace dual_arm::test {

/// 读取默认配置 + 场景（默认 lift）；测试默认关闭日志和扰动（需要的测试自己打开）
inline SimConfig testConfig(std::vector<std::string> overrides = {}, const std::string& scene = "lift") {
  overrides.insert(overrides.begin(), {"log.enabled=false", "disturbances=[]"});
  return loadConfig("config/default.yaml", overrides, scene);
}

inline const std::vector<std::string>& allScenes() {
  static const std::vector<std::string> s{"lift", "slot", "assembly", "assembly_simple"};
  return s;
}

/// 在关节限位内（留 10% 余量）随机采样一个构型
inline Vector7d randomJointConfig(std::mt19937& rng, const Vector7d& lo, const Vector7d& hi) {
  std::uniform_real_distribution<double> u(0.1, 0.9);
  Vector7d q;
  for (int j = 0; j < kArmDof; ++j) q[j] = lo[j] + u(rng) * (hi[j] - lo[j]);
  return q;
}

inline Vector7d randomVector7(std::mt19937& rng, double scale) {
  std::uniform_real_distribution<double> u(-scale, scale);
  Vector7d v;
  for (int j = 0; j < kArmDof; ++j) v[j] = u(rng);
  return v;
}

/// 抓取点 site 的世界坐标（plant）
inline Vector3d graspPoint(const SimEnv& env, Arm a) {
  const int g = env.indices()[a].grasp_site;
  const mjtNum* p = env.data()->site_xpos + 3 * g;
  return Vector3d(p[0], p[1], p[2]);
}

/// 两个 weld wrench（抓取点）换到物体中心后求和 = 两臂施加给物体的合 wrench（两手抓同一物体时）
inline Wrench sumWeldWrenchAtObjectCenter(const SimEnv& env, const DualArmState& s) {
  Wrench sum = Wrench::Zero();
  for (Arm a : kArms) {
    sum += shiftWrenchRefPoint(s.arm(a).weld_wrench, graspPoint(env, a), s.object.pose.p);
  }
  return sum;
}

}  // namespace dual_arm::test
