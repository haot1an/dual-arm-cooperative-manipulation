#pragma once
/**
 * @file mujoco_utils.hpp
 * @brief MuJoCo 相关的公共工具：按基座位姿 / 手指开度编译场景模型、按 SceneSpec 查找索引、补全 SceneSpec。
 *
 * SimEnv（仿真 plant）、MujocoRobotModel（控制器模型）、CollisionModel（距离查询）各自调用
 * loadSceneModel() 得到**独立的 mjModel**：plant 使用含标定误差的真实基座位姿，控制器使用名义位姿。
 */
#include "dual_arm/scene_spec.hpp"
#include "dual_arm/types.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace dual_arm {

struct MjModelDeleter {
  void operator()(mjModel* m) const { mj_deleteModel(m); }
};
struct MjDataDeleter {
  void operator()(mjData* d) const { mj_deleteData(d); }
};
struct MjSpecDeleter {
  void operator()(mjSpec* s) const { mj_deleteSpec(s); }
};
using MjModelPtr = std::unique_ptr<mjModel, MjModelDeleter>;
using MjDataPtr = std::unique_ptr<mjData, MjDataDeleter>;
using MjSpecPtr = std::unique_ptr<mjSpec, MjSpecDeleter>;

/// 单臂相关的 id / 地址（名字加前缀 left_ / right_ 后在模型中查找）
struct ArmIndices {
  std::array<int, kArmDof> joint_id{};     ///< joint1..joint7
  std::array<int, kArmDof> actuator_id{};  ///< actuator1..actuator7（力矩电机）
  int qpos_adr = -1;     ///< 7 个关节在 qpos 中连续存放的起始地址
  int dof_adr = -1;      ///< 7 个关节在 qvel / 自由度中连续存放的起始地址
  int base_body = -1;    ///< left_base / right_base（安装 body，位姿可由 config 修改）
  int hand_body = -1;    ///< hand（Franka Hand）：夹爪 weld 的 body1，F/T 传感器所在 body
  int ee_site = -1;      ///< ee_site（TCP：两指之间、法兰前方 0.1034 m）
  int ft_site = -1;      ///< ft_site（腕部 F/T 传感器安装点，link7 与 hand 的交界面）
  int ft_force_adr = -1;   ///< 力传感器在 sensordata 中的地址（3 维）
  int ft_torque_adr = -1;  ///< 力矩传感器在 sensordata 中的地址（3 维）
  int grasp_site = -1;   ///< 抓取点 site（SceneSpec.grasp.site）
  int grasp_body = -1;   ///< 被抓 body（物体或工具，必须带 free joint）
  int grasp_qpos_adr = -1, grasp_dof_adr = -1;  ///< 被抓 body 的 free joint 地址
  int weld_eq = -1;      ///< 夹爪 weld（hand ↔ 被抓 body）
};

struct ScrewIndices {
  bool valid = false;
  int hinge_joint = -1, slide_joint = -1;
  int hinge_qpos = -1, hinge_dof = -1, slide_qpos = -1, slide_dof = -1;
  int body1 = -1, body2 = -1;  ///< 螺纹孔所在 body / 螺钉 body
};

struct SceneIndices {
  std::array<ArmIndices, kNumArms> arm;
  int object_body = -1;
  int object_qpos_adr = -1;  ///< free joint：pos(3) + quat(4, w x y z)
  int object_dof_adr = -1;   ///< free joint：线速度(3, 世界系) + 角速度(3, body 局部系)
  std::vector<int> rigid_welds;  ///< SceneSpec 中全部 rigid 约束对应的 weld equality id
  ScrewIndices screw;

  const ArmIndices& operator[](Arm a) const { return arm[armIndex(a)]; }
};

/// 解析场景 XML 并覆盖：left_base / right_base 位姿、两臂的手指开度（finger_opening < 0 表示不改）、timestep。
/// 返回未编译的 mjSpec（可继续修改后用 compileSpec 编译，可多次编译）。失败抛异常。
MjSpecPtr loadSceneSpec(const std::string& xml_path, const std::array<Pose, kNumArms>& base_poses,
                        const std::array<double, kNumArms>& finger_openings, double timestep);
/// 编译 mjSpec；失败抛异常（what 用于错误信息）
MjModelPtr compileSpec(mjSpec* spec, const std::string& what);

/// loadSceneSpec + （可选）spec_edit + compileSpec。
MjModelPtr loadSceneModel(const std::string& xml_path, const std::array<Pose, kNumArms>& base_poses,
                          const std::array<double, kNumArms>& finger_openings, double timestep,
                          const std::function<void(mjSpec*)>& spec_edit = nullptr);

/// 查找所有索引并检查模型结构（关节连续、执行器是 motor、weld 连接正确等）。失败抛 std::runtime_error。
SceneIndices findSceneIndices(const mjModel* m, const SceneSpec& scene);

/// 用编译后的模型补全 SceneSpec 的数值字段（物体质量 / 惯量 / 尺寸、抓取点位姿与所属 body），
/// 并检查 SceneSpec 中的名字都存在。
void completeSceneSpec(SceneSpec& scene, const mjModel* m);

/// obstacles 中的名字 → geom id 列表：left_arm / right_arm（该臂除 link0 外的全部碰撞体）、object（主物体）、
/// body 名（该 body 的全部碰撞体）、geom 名。只返回参与碰撞的 geom（contype 或 conaffinity 非 0）。
std::vector<int> resolveGeomGroup(const mjModel* m, const SceneSpec& scene, const std::string& name);

/// 读取 MuJoCo 数组中的位姿（pos[3] + quat[4] w,x,y,z 或 xmat[9]）
Pose poseFromMj(const mjtNum* pos, const mjtNum* quat_wxyz);
Pose poseFromMjMat(const mjtNum* pos, const mjtNum* xmat);

}  // namespace dual_arm
