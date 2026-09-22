#include "dual_arm/logger.hpp"

#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace dual_arm {
namespace {

namespace fs = std::filesystem;

const char* const kWrenchSuffix[6] = {"fx", "fy", "fz", "mx", "my", "mz"};
const char* const kPoseSuffix[7] = {"x", "y", "z", "qw", "qx", "qy", "qz"};
const char* const kArmPrefix[2] = {"l", "r"};

// 把一行的所有数值按列顺序依次追加（与 columnNames() 的顺序严格一致）
struct RowPacker {
  double* out;
  std::size_t cap;
  std::size_t n = 0;
  void add(double v) {
    if (n < cap) out[n] = v;
    ++n;  // 超出容量时只计数，由调用方检查
  }
  template <typename Derived>
  void add(const Eigen::MatrixBase<Derived>& v) {
    for (Eigen::Index i = 0; i < v.size(); ++i) add(static_cast<double>(v(i)));
  }
  void add(const Pose& p) {
    add(p.p);
    add(p.q.w());
    add(p.q.x());
    add(p.q.y());
    add(p.q.z());
  }
};

}  // namespace

std::vector<std::string> CsvLogger::columnNames() {
  std::vector<std::string> c;
  c.push_back("t");
  for (const char* q : {"q", "dq", "tau"}) {
    for (const char* a : kArmPrefix) {
      for (int j = 1; j <= kArmDof; ++j) c.push_back(std::string(a) + "_" + q + std::to_string(j));
    }
  }
  for (const char* w : {"ft", "ftw", "weld"}) {
    for (const char* a : kArmPrefix) {
      for (const char* s : kWrenchSuffix) c.push_back(std::string(a) + "_" + w + "_" + s);
    }
  }
  for (const char* a : kArmPrefix) {
    for (const char* s : kPoseSuffix) c.push_back(std::string(a) + "_ee_" + s);
  }
  for (const char* s : kPoseSuffix) c.push_back(std::string("obj_") + s);
  for (const char* s : {"vx", "vy", "vz", "wx", "wy", "wz"}) c.push_back(std::string("obj_") + s);
  for (const char* s : kPoseSuffix) c.push_back(std::string("ref_") + s);
  c.push_back("ref_screw");
  for (const char* s : kWrenchSuffix) c.push_back(std::string("dist_") + s);
  for (const char* s : {"x", "y", "z", "rx", "ry", "rz"}) c.push_back(std::string("obj_err_") + s);
  for (const char* a : kArmPrefix) {
    for (const char* s : kWrenchSuffix) c.push_back(std::string("int_") + a + "_" + s);
  }
  c.push_back("assembly_phase");
  c.push_back("tightening_tau_ref");
  c.push_back("tightening_tau_meas");
  c.push_back("preload_ref");
  c.push_back("preload_meas");
  c.push_back("governor_phase");
  c.push_back("governor_offset");
  c.push_back("governor_virtual_time");
  c.push_back("t_ctrl_us");
  c.push_back("t_step_us");
  return c;
}

CsvLogger::~CsvLogger() { close(); }

bool CsvLogger::open(const std::string& path) {
  close();
  file_ = std::fopen(path.c_str(), "w");
  if (!file_) return false;
  path_ = path;
  io_buffer_.resize(1 << 20);
  std::setvbuf(file_, io_buffer_.data(), _IOFBF, io_buffer_.size());
  auto names = columnNames();
  names.insert(names.end(), extra_names_.begin(), extra_names_.end());
  values_.assign(names.size(), 0.0);
  for (std::size_t i = 0; i < names.size(); ++i) {
    std::fputs(names[i].c_str(), file_);
    std::fputc(i + 1 < names.size() ? ',' : '\n', file_);
  }
  return true;
}

void CsvLogger::write(const LogRow& row) {
  if (!file_ || !row.step) return;
  const DualArmState& s = *row.step;
  RowPacker p{values_.data(), values_.size()};
  p.add(s.t);
  for (int k = 0; k < 3; ++k) {
    for (Arm a : kArms) {
      const ArmState& as = s.arm(a);
      p.add(k == 0 ? as.q : (k == 1 ? as.dq : as.tau));
    }
  }
  for (Arm a : kArms) p.add(s.arm(a).ft_raw);
  for (Arm a : kArms) p.add(s.arm(a).ft_ee_world);
  for (Arm a : kArms) p.add(s.arm(a).weld_wrench);
  for (Arm a : kArms) p.add(s.arm(a).ee_pose);
  p.add(s.object.pose);
  p.add(s.object.twist);
  p.add(row.object_ref);
  p.add(row.screw_ref);
  p.add(s.disturbance);
  p.add(row.object_error);
  p.add(row.internal_wrench);
  p.add(row.assembly_phase);
  p.add(row.tightening_torque_ref);
  p.add(row.tightening_torque_meas);
  p.add(row.preload_ref);
  p.add(row.preload_meas);
  p.add(row.governor_phase);
  p.add(row.governor_offset);
  p.add(row.governor_virtual_time);
  p.add(row.ctrl_time_us);
  p.add(row.step_time_us);
  for (std::size_t i = 0; i < extra_names_.size(); ++i) p.add(row.extra ? row.extra[i] : 0.0);
  if (p.n != values_.size()) {
    throw std::logic_error("CsvLogger: packed value count does not match column count");
  }
  for (std::size_t i = 0; i < p.n; ++i) {
    std::fprintf(file_, "%.9g", values_[i]);
    std::fputc(i + 1 < p.n ? ',' : '\n', file_);
  }
}

void CsvLogger::close() {
  if (file_) {
    std::fclose(file_);
    file_ = nullptr;
  }
}

std::string makeRunDirectory(const std::string& log_root, const std::string& tag) {
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  localtime_r(&now, &tm);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
  fs::path dir = fs::path(log_root) / (std::string(stamp) + "_" + tag);
  for (int k = 1; fs::exists(dir); ++k) {
    dir = fs::path(log_root) / (std::string(stamp) + "_" + tag + "_" + std::to_string(k));
  }
  fs::create_directories(dir);
  return dir.string();
}

void writeTextFile(const std::string& path, const std::string& content) {
  std::ofstream f(path);
  if (!f) throw std::runtime_error("cannot write '" + path + "'");
  f << content;
}

}  // namespace dual_arm
