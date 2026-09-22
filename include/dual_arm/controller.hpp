#pragma once
/**
 * @file controller.hpp
 * @brief 控制器抽象基类。
 *
 * 约定：
 *  - compute() 每个控制周期调用一次，返回 (τ_left, τ_right)，单位 N·m；
 *    SimEnv 会再按 ctrlrange 截断，所以饱和处理可选，但建议控制器内部自己处理（QP）。
 *  - 控制器持有 RobotModel（名义模型），并在 compute() 开头调用 model().update(state)。
 *  - 场景信息只通过 scene()（SceneSpec）获得：主物体参数、抓取点、闭链约束列表及其相对自由度、
 *    航点、需要避障的 geom 对。控制器代码中不得硬编码布局 / 物体 / 约束类型。
 *  - 控制器**只应使用** state 中的测量量（q, dq, 腕部 F/T, 以及视情况使用 object / screw），
 *    末端位姿等运动学量应通过 RobotModel（名义模型）计算；state 中的 ee_pose / weld_wrench
 *    是仿真真值，只用于评估。否则标定误差实验就失去意义。
 *  - compute() 内不要做动态内存分配（Eigen 用定长矩阵，缓冲区在构造/reset 中分配）。
 *  - 该接口不依赖 MuJoCo，以后可以原样放进 ROS2 的控制器插件里。
 */
#include "dual_arm/robot_model.hpp"
#include "dual_arm/scene_spec.hpp"
#include "dual_arm/types.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace dual_arm {

class Controller {
 public:
  explicit Controller(std::shared_ptr<RobotModel> model) : model_(std::move(model)) {
    if (!model_) throw std::invalid_argument("Controller: model is null");
  }
  virtual ~Controller() = default;

  virtual const char* name() const = 0;

  /// 仿真开始（t = 0）时调用一次，可用于记录初始构型、初始物体位姿等。
  virtual void reset(const DualArmState& /*initial_state*/) {}

  /// 计算两臂关节力矩 (τ_left, τ_right)。
  virtual std::pair<Vector7d, Vector7d> compute(const DualArmState& state, double t) = 0;

  RobotModel& model() { return *model_; }
  const RobotModel& model() const { return *model_; }
  /// 场景描述（= model().scene()）
  const SceneSpec& scene() const { return model_->scene(); }

 protected:
  std::shared_ptr<RobotModel> model_;
};

}  // namespace dual_arm
