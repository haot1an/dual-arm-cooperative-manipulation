"""双臂场景的离线运动学工具（基座标定、场景几何设计、可行性验算共用）。

依赖：系统 python3 的 numpy / scipy / pyyaml，以及 CMake 编译出的 build/libdual_arm_mjshim.so
（tools/mjshim.cpp，链接与仿真相同的 MuJoCo 3.12；先 `cmake --build build` 再运行工具）。

与 C++ 端读取同一份配置：config/default.yaml（布局等公共参数）+ config/scenes/<name>.yaml（场景）。
约定与 C++ 一致：
  - 基座位姿 = layout.{left,right}.base_pos / base_rpy_deg，R = Rz(yaw) Ry(pitch) Rx(roll)；
  - 手指开度由场景的 grasps.<arm>.finger_opening 决定（加载时写入 <arm>_left_finger / _right_finger 的 pos）；
  - TCP = <arm>_ee_site；抓取点 site 的位姿（在其所属 body 系中）与 TCP 重合时即为抓取成功。
"""
from __future__ import annotations

import copy
import ctypes
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

ROOT = Path(__file__).resolve().parents[1]
ARMS = ("left", "right")
TCP_OFFSET = 0.1034  # 法兰到 TCP 的距离（Franka 约定）
OBJ_BODY, OBJ_JOINT, OBJ_GEOM, OBJ_SITE, OBJ_CAMERA = 1, 3, 5, 6, 7

# ---------------------------------------------------------------------------
# MuJoCo 3.12 shim（ctypes）
# ---------------------------------------------------------------------------
_LIB = None


def lib():
    global _LIB
    if _LIB is None:
        path = ROOT / "build" / "libdual_arm_mjshim.so"
        if not path.exists():
            raise SystemExit(f"{path} 不存在：先运行 cmake -B build && cmake --build build")
        L = ctypes.CDLL(str(path))
        P, D, I = ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
        L.shim_load.restype = P
        L.shim_load.argtypes = [ctypes.c_char_p, D, D, ctypes.c_char_p, ctypes.c_int]
        L.shim_free.argtypes = [P]
        L.shim_size.restype = ctypes.c_int
        L.shim_size.argtypes = [P, ctypes.c_char_p]
        L.shim_name2id.restype = ctypes.c_int
        L.shim_name2id.argtypes = [P, ctypes.c_int, ctypes.c_char_p]
        L.shim_id2name.restype = ctypes.c_char_p
        L.shim_id2name.argtypes = [P, ctypes.c_int, ctypes.c_int]
        for n in ("qpos", "xpos", "xmat", "site_xpos", "site_xmat", "geom_xpos", "geom_xmat", "jnt_range",
                  "geom_size", "site_pos", "site_quat", "key_qpos"):
            f = getattr(L, f"shim_{n}")
            f.restype = D
            f.argtypes = [P]
        for n in ("jnt_qposadr", "jnt_dofadr", "body_jntadr", "geom_bodyid", "geom_contype", "geom_conaffinity",
                  "geom_type", "site_bodyid"):
            f = getattr(L, f"shim_{n}")
            f.restype = I
            f.argtypes = [P]
        L.shim_kinematics.argtypes = [P]
        L.shim_jac_site.argtypes = [P, ctypes.c_int, D, D]
        L.shim_geom_distance.restype = ctypes.c_double
        L.shim_geom_distance.argtypes = [P, ctypes.c_int, ctypes.c_int, ctypes.c_double, D]
        L.shim_min_distance.restype = ctypes.c_double
        L.shim_min_distance.argtypes = [P, I, ctypes.c_int, I, ctypes.c_int, ctypes.c_double, I]
        _LIB = L
    return _LIB


def _dptr(a):
    return a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


def _iptr(a):
    return a.ctypes.data_as(ctypes.POINTER(ctypes.c_int))


class Model:
    """MuJoCo 3.12 模型 + 数据（只读模型数组与可写 qpos 都是 numpy 视图）"""

    def __init__(self, xml: Path, base: np.ndarray, fingers: np.ndarray):
        L = lib()
        err = ctypes.create_string_buffer(1024)
        self.h = L.shim_load(str(xml).encode(), _dptr(np.ascontiguousarray(base, float)),
                             _dptr(np.ascontiguousarray(fingers, float)), err, 1024)
        if not self.h:
            raise RuntimeError(f"load {xml}: {err.value.decode()}")
        sz = lambda w: L.shim_size(self.h, w.encode())
        self.nq, self.nv, self.njnt, self.nbody, self.ngeom, self.nsite, self.nkey = (
            sz(w) for w in ("nq", "nv", "njnt", "nbody", "ngeom", "nsite", "nkey"))
        A = np.ctypeslib.as_array
        self.qpos = A(L.shim_qpos(self.h), shape=(self.nq,))
        self.xpos = A(L.shim_xpos(self.h), shape=(self.nbody, 3))
        self.xmat = A(L.shim_xmat(self.h), shape=(self.nbody, 9))
        self.site_xpos = A(L.shim_site_xpos(self.h), shape=(self.nsite, 3))
        self.site_xmat = A(L.shim_site_xmat(self.h), shape=(self.nsite, 9))
        self.geom_xpos = A(L.shim_geom_xpos(self.h), shape=(self.ngeom, 3))
        self.geom_xmat = A(L.shim_geom_xmat(self.h), shape=(self.ngeom, 9))
        self.jnt_range = A(L.shim_jnt_range(self.h), shape=(self.njnt, 2))
        self.geom_size = A(L.shim_geom_size(self.h), shape=(self.ngeom, 3))
        self.site_pos = A(L.shim_site_pos(self.h), shape=(self.nsite, 3))
        self.site_quat = A(L.shim_site_quat(self.h), shape=(self.nsite, 4))
        self.jnt_qposadr = A(L.shim_jnt_qposadr(self.h), shape=(self.njnt,))
        self.jnt_dofadr = A(L.shim_jnt_dofadr(self.h), shape=(self.njnt,))
        self.body_jntadr = A(L.shim_body_jntadr(self.h), shape=(self.nbody,))
        self.geom_bodyid = A(L.shim_geom_bodyid(self.h), shape=(self.ngeom,))
        self.geom_contype = A(L.shim_geom_contype(self.h), shape=(self.ngeom,))
        self.geom_conaffinity = A(L.shim_geom_conaffinity(self.h), shape=(self.ngeom,))
        self.geom_type = A(L.shim_geom_type(self.h), shape=(self.ngeom,))
        self.site_bodyid = A(L.shim_site_bodyid(self.h), shape=(self.nsite,))
        self._jp = np.zeros(3 * self.nv)
        self._jr = np.zeros(3 * self.nv)

    def __del__(self):
        if getattr(self, "h", None):
            lib().shim_free(self.h)

    def id(self, obj, name):
        return lib().shim_name2id(self.h, obj, name.encode())

    def name(self, obj, i):
        n = lib().shim_id2name(self.h, obj, int(i))
        return n.decode() if n else None

    def kinematics(self):
        lib().shim_kinematics(self.h)

    def jac_site(self, site):
        lib().shim_jac_site(self.h, int(site), _dptr(self._jp), _dptr(self._jr))
        return self._jp.reshape(3, self.nv), self._jr.reshape(3, self.nv)

    def geom_distance(self, g1, g2, distmax=0.5):
        return lib().shim_geom_distance(self.h, int(g1), int(g2), distmax, None)

    def min_distance(self, ga, gb, distmax=0.5):
        a = np.ascontiguousarray(ga, dtype=np.int32)
        b = np.ascontiguousarray(gb, dtype=np.int32)
        pair = np.zeros(2, dtype=np.int32)
        dist = lib().shim_min_distance(self.h, _iptr(a), len(a), _iptr(b), len(b), distmax, _iptr(pair))
        return dist, (int(pair[0]), int(pair[1]))


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(scene: str | None = None) -> dict:
    """返回 {'root': default.yaml(已合并场景 overrides), 'scene': 场景 yaml, 'name': 场景名}"""
    root = yaml.safe_load((ROOT / "config" / "default.yaml").read_text())
    name = scene or root.get("scene", "lift")
    scene_path = ROOT / root.get("scene_dir", "config/scenes") / f"{name}.yaml"
    sc = yaml.safe_load(scene_path.read_text())
    root = deep_merge(root, sc.get("overrides") or {})
    return {"root": root, "scene": sc, "name": name}


def rpy_matrix(rpy_deg) -> np.ndarray:
    """R = Rz(yaw) Ry(pitch) Rx(roll)（与 C++ quatFromRpyDeg 一致）"""
    return Rot.from_euler("ZYX", [rpy_deg[2], rpy_deg[1], rpy_deg[0]], degrees=True).as_matrix()


def quat_wxyz(R) -> np.ndarray:
    x, y, z, w = Rot.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def pose(p, R):
    return np.asarray(p, float), np.asarray(R, float)


def compose(A, B):
    return A[0] + A[1] @ B[0], A[1] @ B[1]


def inverse(A):
    return -A[1].T @ A[0], A[1].T


def layout_yaw_deg(layout, arm):
    return layout[arm]["base_rpy_deg"][2]


def make_layout(shoulder: float, inward_deg: float, y: float = 0.0, z: float = 0.75) -> dict:
    """同侧并排布局：左臂在 +x，yaw = −90° − 内偏角；右臂在 −x，yaw = −90° + 内偏角"""
    return {"left": {"base_pos": [shoulder / 2, y, z], "base_rpy_deg": [0.0, 0.0, -90.0 - inward_deg]},
            "right": {"base_pos": [-shoulder / 2, y, z], "base_rpy_deg": [0.0, 0.0, -90.0 + inward_deg]}}


# ---------------------------------------------------------------------------
# 场景
# ---------------------------------------------------------------------------
@dataclass
class Scene:
    cfg: dict
    m: Model
    layout: dict
    qadr: dict = field(default_factory=dict)
    dofadr: dict = field(default_factory=dict)
    tcp: dict = field(default_factory=dict)
    lo: np.ndarray = None
    hi: np.ndarray = None
    arm_geoms: dict = field(default_factory=dict)

    @classmethod
    def load(cls, cfg: dict, layout: dict | None = None, model_path: Path | None = None) -> "Scene":
        sc = cfg["scene"]
        lay = layout or cfg["root"]["layout"]
        base = np.zeros((2, 7))
        for i, arm in enumerate(ARMS):
            base[i, :3] = lay[arm]["base_pos"]
            base[i, 3:] = quat_wxyz(rpy_matrix(lay[arm]["base_rpy_deg"]))
        fingers = np.array([sc["grasps"][a]["finger_opening"] for a in ARMS], float)
        m = Model(model_path or (ROOT / sc["model"]), base, fingers)
        s = cls(cfg=cfg, m=m, layout=lay)
        for arm in ARMS:
            j1 = m.id(OBJ_JOINT, f"{arm}_joint1")
            s.qadr[arm] = m.jnt_qposadr[j1]
            s.dofadr[arm] = m.jnt_dofadr[j1]
            s.tcp[arm] = m.id(OBJ_SITE, f"{arm}_ee_site")
            s.arm_geoms[arm] = s.collision_geoms(lambda b, p=f"{arm}_": b.startswith(p) and not b.endswith("link0"))
        jids = [m.id(OBJ_JOINT, f"left_joint{k}") for k in range(1, 8)]
        s.lo = np.array([m.jnt_range[j, 0] for j in jids])
        s.hi = np.array([m.jnt_range[j, 1] for j in jids])
        return s

    # ---- 名字 → id ----
    def gid(self, name):
        g = self.m.id(OBJ_GEOM, name)
        if g < 0:
            raise KeyError(f"no geom {name}")
        return g

    def bid(self, name):
        b = self.m.id(OBJ_BODY, name)
        if b < 0:
            raise KeyError(f"no body {name}")
        return b

    def collision_geoms(self, body_pred):
        out = []
        for g in range(self.m.ngeom):
            bname = self.m.name(OBJ_BODY, self.m.geom_bodyid[g]) or ""
            if body_pred(bname) and (self.m.geom_contype[g] or self.m.geom_conaffinity[g]):
                out.append(g)
        return out

    def geoms_of_bodies(self, names):
        names = set(names)
        return self.collision_geoms(lambda b: b in names)

    def resolve(self, name):
        """obstacles 中的名字 → geom 列表（left_arm / right_arm / object / body 名 / geom 名）"""
        if name in ("left_arm", "right_arm"):
            return list(self.arm_geoms[name.split("_")[0]])
        if name == "object":
            return self.geoms_of_bodies([self.cfg["scene"]["object"]["body"]])
        if self.m.id(OBJ_BODY, name) >= 0:
            return self.geoms_of_bodies([name])
        return [self.gid(name)]

    def gname(self, g):
        n = self.m.name(OBJ_GEOM, g)
        return n if n else f"{self.m.name(OBJ_BODY, self.m.geom_bodyid[g])}#{g}"

    # ---- 状态 ----
    def set_arm(self, arm, q):
        self.m.qpos[self.qadr[arm]:self.qadr[arm] + 7] = q

    def set_free_body(self, body, T):
        j = self.m.body_jntadr[self.bid(body)]
        a = self.m.jnt_qposadr[j]
        self.m.qpos[a:a + 3] = T[0]
        self.m.qpos[a + 3:a + 7] = quat_wxyz(T[1])

    def forward(self):
        self.m.kinematics()

    def tcp_pose(self, arm):
        s = self.tcp[arm]
        return self.m.site_xpos[s].copy(), self.m.site_xmat[s].reshape(3, 3).copy()

    def site_in_body(self, site):
        sid = self.m.id(OBJ_SITE, site)
        if sid < 0:
            raise KeyError(f"no site {site}")
        w, x, y, z = self.m.site_quat[sid]
        return self.m.site_pos[sid].copy(), Rot.from_quat([x, y, z, w]).as_matrix()

    def site_body(self, site):
        return self.m.name(OBJ_BODY, self.m.site_bodyid[self.m.id(OBJ_SITE, site)])

    def body_pose(self, body):
        b = self.bid(body)
        return self.m.xpos[b].copy(), self.m.xmat[b].reshape(3, 3).copy()

    def jacobian(self, arm):
        jp, jr = self.m.jac_site(self.tcp[arm])
        d0 = self.dofadr[arm]
        return np.vstack([jp[:, d0:d0 + 7], jr[:, d0:d0 + 7]])

    # ---- IK ----
    def ik(self, arm, T_des, seeds, w_rot=0.3, w_reg=0.05, q7_target=None, free_yaw=False, tol=1e-6,
           q_prefer=None):
        """IK。返回 (q, 残差) 或 (None, inf)。

        冗余度（7 自由度 − 6 维任务 = 1）的处理：
          q_prefer 为 None：最小化 Σ((q−mid)/(range/2))^8（近似“最大化最小裕度”，用于选抓取构型）；
          q_prefer 给定  ：最小化 ‖q − q_prefer‖²（最小关节运动，用于沿路径连续跟踪，模拟真实控制器的连续运动）。
        free_yaw=True 时只约束 TCP 的 z 轴方向（用于圆柱形工具）。q7_target 给定时把 q7 钉在该值附近。
        """
        lo, hi = self.lo, self.hi
        mid, half = (lo + hi) / 2, (hi - lo) / 2
        p_des, R_des = T_des

        def task(q):
            self.set_arm(arm, q)
            self.forward()
            p, R = self.tcp_pose(arm)
            rot = np.cross(R[:, 2], R_des[:, 2]) if free_yaw else Rot.from_matrix(R_des @ R.T).as_rotvec()
            r = [p_des - p, w_rot * rot]
            if q7_target is not None:
                r.append([2.0 * (q[6] - q7_target)])
            return np.concatenate(r)

        def full(q, w):
            if q_prefer is not None:
                return np.concatenate([task(q), w * (q - q_prefer) / half])
            return np.concatenate([task(q), w * ((q - mid) / half) ** 4])

        best = None
        for s in seeds:
            s = np.clip(np.asarray(s, float), lo + 1e-3, hi - 1e-3)
            r = least_squares(full, s, args=(w_reg,), bounds=(lo + 1e-5, hi - 1e-5), max_nfev=300)
            r = least_squares(full, r.x, args=(w_reg * 0.02,), bounds=(lo + 1e-5, hi - 1e-5), max_nfev=300)
            r2 = least_squares(task, r.x, bounds=(lo + 1e-6, hi - 1e-6), max_nfev=300, xtol=1e-12, ftol=1e-12,
                               gtol=1e-12)
            err = np.linalg.norm(task(r2.x)[:6])
            if err < tol:
                sc = np.min(np.minimum(r2.x - lo, hi - r2.x) / (hi - lo))
                if best is None or sc > best[1] + 1e-3:
                    best = (r2.x.copy(), sc, err)
                if sc > 0.2:
                    break
        return (best[0], best[2]) if best else (None, math.inf)

    # ---- 指标 ----
    def arm_metrics(self, arm, q):
        self.set_arm(arm, q)
        self.forward()
        J = self.jacobian(arm)
        sv = np.linalg.svd(J, compute_uv=False)
        svv = np.linalg.svd(J[:3], compute_uv=False)
        margin = np.minimum(q - self.lo, self.hi - q)
        return dict(q=np.array(q, float), margin=margin, margin_n=margin / (self.hi - self.lo), smin=sv[-1],
                    cond=sv[0] / sv[-1], smin_v=svv[-1], cond_v=svv[0] / svv[-1])

    def min_distance(self, geoms_a, geoms_b, distmax=0.5):
        return self.m.min_distance(geoms_a, geoms_b, distmax)


MIRROR = np.array([-1, 1, -1, 1, -1, 1, -1], float)


def mirror_q(q):
    """把一条臂的构型镜像到另一条臂（两基座关于世界 yz 平面镜像对称时有效）：
    关于 Panda 零位臂平面的反射把 joint1/3/5/7（轴在臂平面内）取反，joint2/4/6（轴垂直于臂平面）不变；
    但 hand 以 −45° 装在 link7 上，这个安装角不是镜像对称的，所以 q7' = −q7 + π/2
    （夹爪绕 z 转 180° 对称，再按需加减 π 落回限位内）。"""
    out = MIRROR * np.asarray(q, float)
    out[6] += math.pi / 2
    if out[6] > 2.8973:
        out[6] -= math.pi
    return out


def ik_candidates(S, arm, T, seeds, near_seeds=(), max_n=12):
    """从多组种子求 IK，返回互不相同（关节空间距离 > 0.3 rad）的解，按最小裕度降序。
    seeds 用“最大化裕度”的冗余解析；near_seeds 用“离种子最近”的解（例如另一条臂的镜像解）。"""
    sols = []

    def add(q):
        if q is not None and all(np.linalg.norm(q - o) > 0.3 for o in sols):
            sols.append(q)

    for s in near_seeds:
        add(S.ik(arm, T, [s], w_reg=0.2, q_prefer=np.asarray(s, float))[0])
    for s in seeds:
        add(S.ik(arm, T, [s])[0])
    sols.sort(key=lambda q: -np.min(np.minimum(q - S.lo, S.hi - q) / (S.hi - S.lo)))
    return sols[:max_n]


def track_path(S, arm, targets, q0):
    """从 q0 出发沿 TCP 目标序列做最小关节运动跟踪；返回解列表或 None"""
    sols = [q0]
    for T in targets[1:]:
        q, _ = S.ik(arm, T, [sols[-1]], w_reg=0.2, q_prefer=sols[-1])
        if q is None:
            return None
        if np.max(np.abs(q - sols[-1])) > 0.8:  # 连续跟踪不允许大跳变
            return None
        sols.append(q)
    return sols


def path_quality(S, sols, arm=None):
    worst_margin = min(np.min(np.minimum(q - S.lo, S.hi - q) / (S.hi - S.lo)) for q in sols)
    worst_cond = max(S.arm_metrics(arm, q)["cond"] for q in sols) if arm else 0.0
    return worst_margin - 0.005 * worst_cond


def best_path(S, arm, targets, extra_seeds=()):
    """尝试多个起始构型，返回最坏裕度最好的连续路径 (sols, quality) 或 (None, -inf)。
    起始构型候选 = 另一条臂解的镜像 + “自然姿态”种子（肘部朝上、q1/q3/q5 ≈ 0）的最近解 + 最大裕度解。
    只用最大裕度解时，冗余自运动会把 q1/q3/q5 拧成大角度互相抵消的“扭曲”构型（裕度并不更好）。"""
    best = (None, -math.inf)
    for q0 in ik_candidates(S, arm, targets[0], SEEDS, near_seeds=tuple(extra_seeds) + tuple(NATURAL_SEEDS)):
        sols = track_path(S, arm, targets, q0)
        if sols is None:
            continue
        qual = path_quality(S, sols, arm)
        if qual > best[1]:
            best = (sols, qual)
    return best


NATURAL_SEEDS = [np.array(s) for s in (
    [0, 0.3, 0, -2.2, 0, 2.5, 0.785], [0, 0.3, 0, -2.2, 0, 2.5, -0.785],
    [0, 0.5, 0, -2.0, 0, 2.5, 2.3], [0, 0.5, 0, -2.0, 0, 2.5, -2.3])]

SEEDS = [np.array(s) for s in (
    [0, 0.3, 0, -2.0, 0, 2.3, 0.785], [0, 0.3, 0, -2.0, 0, 2.3, -0.785], [0, -0.3, 0, -2.2, 1.57, 1.6, 0],
    [0, -0.3, 0, -2.2, -1.57, 1.6, 0], [0.5, 0.2, 0.5, -2.0, 1.5, 1.8, 0.5], [-0.5, 0.2, -0.5, -2.0, -1.5, 1.8, -0.5],
    [0, 0.5, 0, -1.8, 0, 2.3, 2.0], [0, 0.5, 0, -1.8, 0, 2.3, -2.0])]


def fmt_q(q):
    return "[" + ", ".join(f"{v:+.3f}" for v in q) + "]"
