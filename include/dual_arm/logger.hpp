#pragma once
/**
 * @file logger.hpp
 * @brief CSV 日志：每个仿真步（或每 N 步）一行，所有字段对齐到同一时刻 t。
 *
 * 列（固定部分共 147 列，列名见 CsvLogger::columnNames()；之后追加 setExtraColumns() 给出的场景列，
 * 例如 SceneMonitor 的 screw_* / roll_* / d_*；scripts/plot_log.py 按列名读取）：
 *   t
 *   {l,r}_q1..7, {l,r}_dq1..7, {l,r}_tau1..7          关节量（tau 为饱和后实际施加值）
 *   {l,r}_ft_{fx,fy,fz,mx,my,mz}                      腕部 F/T 原始读数（ft_site 系）
 *   {l,r}_ftw_{...}                                   F/T 换算：夹爪对物体的 wrench（世界系，TCP，扣夹爪重力）
 *   {l,r}_weld_{...}                                  weld 约束：夹爪（hand）对被抓 body 的 wrench（世界系，抓取点）——真值
 *   {l,r}_ee_{x,y,z,qw,qx,qy,qz}                      ee_site（TCP）真实位姿（世界系）
 *   obj_{x,y,z,qw,qx,qy,qz}, obj_{vx,vy,vz,wx,wy,wz}  主物体真实位姿 / twist（世界系，中心）
 *   ref_{x,y,z,qw,qx,qy,qz}, ref_screw                物体参考位姿、螺钉转角参考 [rad]
 *   dist_{fx,fy,fz,mx,my,mz}                          施加在主物体质心的扰动（世界系）
 *   obj_err_{x,y,z,rx,ry,rz}                          物体位姿误差 [p_ref − p; log(R_ref Rᵀ)]（世界系）
 *   int_{l,r}_{fx,...,mz}                             内力 h_int（由 coop::internalWrenchForLogging 填入）
 *   assembly_phase                                    装配状态机：0 preload, 1 angle, 2 torque, 3 hold
 *   tightening_tau_{ref,meas}                         拧紧力矩参考 / 滤波测量 [N·m]（正值为拧紧）
 *   preload_{ref,meas}                                轴向预紧力参考 / 滤波测量 [N]（正值为压紧）
 *   governor_phase                                    自主避障阶段：0 normal, 1 lift, 2 cross, 3 descend
 *   governor_offset                                   避障参考偏移量 [m]
 *   governor_virtual_time                             参考轨迹的虚拟时间 [s]
 *   t_ctrl_us, t_step_us                              本步控制器计算耗时 / 仿真步耗时 [µs]
 *
 * write() 不做动态内存分配（缓冲区在 open() 时分配）。
 */
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/types.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace dual_arm {

struct LogRow {
  const DualArmState* step = nullptr;          ///< SimEnv::lastStep()
  Pose object_ref;                             ///< 物体参考位姿
  double screw_ref = 0.0;                      ///< 螺钉转角参考
  Vector6d object_error = Vector6d::Zero();    ///< [e_p; e_o]
  Vector12d internal_wrench = Vector12d::Zero();
  double assembly_phase = -1.0;
  double tightening_torque_ref = 0.0;
  double tightening_torque_meas = 0.0;
  double preload_ref = 0.0;
  double preload_meas = 0.0;
  double governor_phase = 0.0;
  double governor_offset = 0.0;
  double governor_virtual_time = 0.0;
  double ctrl_time_us = 0.0;
  double step_time_us = 0.0;
  const double* extra = nullptr;               ///< 场景列的值（个数 = setExtraColumns 的列数）
};

class CsvLogger {
 public:
  CsvLogger() = default;
  ~CsvLogger();
  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;

  /// 在 open() 之前设置追加在固定列之后的场景列名
  void setExtraColumns(std::vector<std::string> names) { extra_names_ = std::move(names); }
  /// 打开文件并写入表头；失败返回 false
  bool open(const std::string& path);
  void write(const LogRow& row);
  void close();
  bool isOpen() const { return file_ != nullptr; }
  const std::string& path() const { return path_; }

  static std::vector<std::string> columnNames();

 private:
  std::FILE* file_ = nullptr;
  std::string path_;
  std::vector<char> io_buffer_;
  std::vector<double> values_;
  std::vector<std::string> extra_names_;
};

/// 创建 <log_root>/<YYYYmmdd_HHMMSS>_<tag>/ 目录并返回其路径（同名已存在时追加 _1, _2 ...）
std::string makeRunDirectory(const std::string& log_root, const std::string& tag);
/// 写文本文件；失败抛 std::runtime_error
void writeTextFile(const std::string& path, const std::string& content);

}  // namespace dual_arm
