// mjshim —— 给 tools/*.py 用的极简 MuJoCo C 接口（ctypes 调用）。
//
// 为什么需要它：本机 conda 环境里的 MuJoCo Python 绑定是 3.8，它的 mj_geomDistance 对“旋转的盒子 /
// 网格 vs 盒子”经常返回错误的 0（3.12 正常），会让基座标定与场景设计里的所有间隙都失真。
// 这个 shim 链接工程实际使用的 MuJoCo 3.12，让离线工具与仿真用同一个版本、同一套加载逻辑：
//   场景 XML → mjSpec → 覆盖 <arm>_base 位姿与两指开度 → 编译（与 C++ 的 loadSceneModel 一致）。
// 由 CMake 编译为 build/libdual_arm_mjshim.so；Python 侧封装见 tools/dual_arm_kin.py。
#include <mujoco/mujoco.h>

#include <cstring>
#include <string>

namespace {

struct Shim {
  mjModel* m = nullptr;
  mjData* d = nullptr;
};

void copyErr(char* err, int n, const std::string& msg) {
  if (err && n > 0) {
    std::strncpy(err, msg.c_str(), static_cast<std::size_t>(n) - 1);
    err[n - 1] = '\0';
  }
}

}  // namespace

extern "C" {

/// base: 2×7（左、右臂：pos xyz + quat wxyz）；fingers: 2（每指开度，m；<0 表示保持 XML 中的值）
void* shim_load(const char* xml, const double* base, const double* fingers, char* err, int err_len) {
  char buf[1024] = "";
  mjSpec* spec = mj_parseXML(xml, nullptr, buf, sizeof(buf));
  if (!spec) {
    copyErr(err, err_len, buf);
    return nullptr;
  }
  const char* arms[2] = {"left", "right"};
  for (int a = 0; a < 2; ++a) {
    const std::string pre(arms[a]);
    mjsBody* b = mjs_findBody(spec, (pre + "_base").c_str());
    if (!b) {
      copyErr(err, err_len, "no body " + pre + "_base");
      mj_deleteSpec(spec);
      return nullptr;
    }
    for (int k = 0; k < 3; ++k) b->pos[k] = base[7 * a + k];
    for (int k = 0; k < 4; ++k) b->quat[k] = base[7 * a + 3 + k];
    b->alt.type = mjORIENTATION_QUAT;
    if (fingers && fingers[a] >= 0) {
      mjsBody* lf = mjs_findBody(spec, (pre + "_left_finger").c_str());
      mjsBody* rf = mjs_findBody(spec, (pre + "_right_finger").c_str());
      if (lf && rf) {
        lf->pos[1] = fingers[a];
        rf->pos[1] = -fingers[a];
      }
    }
  }
  mjModel* m = mj_compile(spec, nullptr);
  if (!m) {
    copyErr(err, err_len, mjs_getError(spec));
    mj_deleteSpec(spec);
    return nullptr;
  }
  mj_deleteSpec(spec);
  auto* s = new Shim;
  s->m = m;
  s->d = mj_makeData(m);
  if (m->nkey > 0) mj_resetDataKeyframe(m, s->d, 0);
  mj_kinematics(m, s->d);
  mj_comPos(m, s->d);
  return s;
}

void shim_free(void* h) {
  auto* s = static_cast<Shim*>(h);
  if (!s) return;
  mj_deleteData(s->d);
  mj_deleteModel(s->m);
  delete s;
}

#define M static_cast<Shim*>(h)->m
#define D static_cast<Shim*>(h)->d

int shim_size(void* h, const char* what) {
  const std::string w(what);
  if (w == "nq") return M->nq;
  if (w == "nv") return M->nv;
  if (w == "njnt") return M->njnt;
  if (w == "nbody") return M->nbody;
  if (w == "ngeom") return M->ngeom;
  if (w == "nsite") return M->nsite;
  if (w == "nkey") return M->nkey;
  return -1;
}

int shim_name2id(void* h, int type, const char* name) { return mj_name2id(M, type, name); }
const char* shim_id2name(void* h, int type, int id) { return mj_id2name(M, type, id); }

double* shim_qpos(void* h) { return D->qpos; }
const double* shim_xpos(void* h) { return D->xpos; }
const double* shim_xmat(void* h) { return D->xmat; }
const double* shim_site_xpos(void* h) { return D->site_xpos; }
const double* shim_site_xmat(void* h) { return D->site_xmat; }
const double* shim_geom_xpos(void* h) { return D->geom_xpos; }
const double* shim_geom_xmat(void* h) { return D->geom_xmat; }

const double* shim_jnt_range(void* h) { return M->jnt_range; }
const int* shim_jnt_qposadr(void* h) { return M->jnt_qposadr; }
const int* shim_jnt_dofadr(void* h) { return M->jnt_dofadr; }
const int* shim_body_jntadr(void* h) { return M->body_jntadr; }
const int* shim_geom_bodyid(void* h) { return M->geom_bodyid; }
const int* shim_geom_contype(void* h) { return M->geom_contype; }
const int* shim_geom_conaffinity(void* h) { return M->geom_conaffinity; }
const int* shim_geom_type(void* h) { return M->geom_type; }
const double* shim_geom_size(void* h) { return M->geom_size; }
const double* shim_site_pos(void* h) { return M->site_pos; }
const double* shim_site_quat(void* h) { return M->site_quat; }
const int* shim_site_bodyid(void* h) { return M->site_bodyid; }
const double* shim_key_qpos(void* h) { return M->key_qpos; }

void shim_kinematics(void* h) {
  mj_kinematics(M, D);
  mj_comPos(M, D);
}

/// jacp, jacr: 3×nv（行主序）
void shim_jac_site(void* h, int site, double* jacp, double* jacr) { mj_jacSite(M, D, jacp, jacr, site); }

double shim_geom_distance(void* h, int g1, int g2, double distmax, double* fromto) {
  return mj_geomDistance(M, D, g1, g2, distmax, fromto);
}

/// 两组 geom 之间的最小距离；pair_out[2] 返回对应 geom id
double shim_min_distance(void* h, const int* ga, int na, const int* gb, int nb, double distmax, int* pair_out) {
  double best = distmax;
  if (pair_out) pair_out[0] = pair_out[1] = -1;
  for (int i = 0; i < na; ++i) {
    for (int j = 0; j < nb; ++j) {
      if (ga[i] == gb[j]) continue;
      const double dist = mj_geomDistance(M, D, ga[i], gb[j], best, nullptr);
      if (dist < best) {
        best = dist;
        if (pair_out) {
          pair_out[0] = ga[i];
          pair_out[1] = gb[j];
        }
      }
    }
  }
  return best;
}

#undef M
#undef D

}  // extern "C"
