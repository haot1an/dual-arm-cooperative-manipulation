#include "dual_arm/scene_spec.hpp"

#include <cmath>

namespace dual_arm {

std::vector<Vector6d> ClosedChainConstraint::relativeMotionBasis() const {
  std::vector<Vector6d> out;
  if (type == ConstraintType::Screw) {
    Vector6d s;
    const Vector3d a = axis.normalized();
    s << (lead / (2.0 * M_PI)) * a, a;  // 转 1 rad 沿轴移动 lead/2π
    out.push_back(s);
  }
  return out;
}

int SceneSpec::relativeDofBetweenHands() const {
  int n = 0;
  for (const auto& c : constraints) n += c.relative_dof;
  return n;
}

const ClosedChainConstraint* SceneSpec::screwConstraint() const {
  for (const auto& c : constraints) {
    if (c.type == ConstraintType::Screw) return &c;
  }
  return nullptr;
}

const ClosedChainConstraint* SceneSpec::constraint(const std::string& n) const {
  for (const auto& c : constraints) {
    if (c.name == n) return &c;
  }
  return nullptr;
}

}  // namespace dual_arm
