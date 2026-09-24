#include "dual_arm/config.hpp"

#include "dual_arm/math_utils.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace dual_arm
{
  namespace
  {

    namespace fs = std::filesystem;

    template <int N>
    Eigen::Matrix<double, N, 1> readVec(const YAML::Node &node, const char *key,
                                        const Eigen::Matrix<double, N, 1> &def)
    {
      const YAML::Node v = node[key];
      if (!v)
        return def;
      if (!v.IsSequence() || static_cast<int>(v.size()) != N)
      {
        throw std::runtime_error(std::string("config: '") + key + "' must be a list of " +
                                 std::to_string(N) + " numbers");
      }
      Eigen::Matrix<double, N, 1> out;
      for (int i = 0; i < N; ++i)
        out[i] = v[i].as<double>();
      return out;
    }

    template <typename T>
    T readScalar(const YAML::Node &node, const char *key, const T &def)
    {
      if (!node)
        return def;
      const YAML::Node v = node[key];
      return v ? v.as<T>() : def;
    }

    template <std::size_t N>
    std::array<double, N> readArray(const YAML::Node &node, const char *key,
                                    const std::array<double, N> &def)
    {
      const YAML::Node v = node[key];
      if (!v)
        return def;
      if (!v.IsSequence() || v.size() != N)
      {
        throw std::runtime_error(std::string("config: '") + key + "' must have " + std::to_string(N) +
                                 " entries");
      }
      std::array<double, N> out{};
      for (std::size_t i = 0; i < N; ++i)
        out[i] = v[i].as<double>();
      return out;
    }

    /// 递归合并：map 逐键合并，其它类型（标量、序列）整体替换
    void deepMerge(YAML::Node base, const YAML::Node &over)
    {
      if (!over || !over.IsMap())
        return;
      for (const auto &kv : over)
      {
        const std::string key = kv.first.as<std::string>();
        if (kv.second.IsMap() && base[key] && base[key].IsMap())
        {
          deepMerge(base[key], kv.second);
        }
        else
        {
          base[key] = YAML::Clone(kv.second);
        }
      }
    }

    /// "a.b.c=value"：沿路径创建/覆盖节点
    void applyOverride(YAML::Node &root, const std::string &expr)
    {
      const auto eq = expr.find('=');
      if (eq == std::string::npos || eq == 0)
      {
        throw std::runtime_error("override '" + expr + "' is not of the form key.path=value");
      }
      const std::string path = expr.substr(0, eq);
      const std::string value = expr.substr(eq + 1);

      std::vector<std::string> keys;
      std::stringstream ss(path);
      for (std::string k; std::getline(ss, k, '.');)
        keys.push_back(k);

      YAML::Node node = root;
      bool existed = true;
      for (std::size_t i = 0; i + 1 < keys.size(); ++i)
      {
        if (!node[keys[i]])
          existed = false;
        YAML::Node child = node[keys[i]];
        node.reset(child); // 注意：yaml-cpp 中 node = child 会改写内容而不是重新绑定
      }
      if (!node[keys.back()])
        existed = false;
      if (!existed)
      {
        std::fprintf(stderr, "[config] note: override key '%s' did not exist in the config file\n",
                     path.c_str());
      }
      node[keys.back()] = YAML::Load(value);
    }

    Pose poseFromYaml(const YAML::Node &n)
    {
      Pose T;
      T.p = readVec<3>(n, "pos", Vector3d::Zero());
      T.q = quatFromRpyDeg(readVec<3>(n, "rpy_deg", Vector3d::Zero()));
      return T;
    }

    Arm armFromString(const std::string &s)
    {
      if (s == "left")
        return Arm::Left;
      if (s == "right")
        return Arm::Right;
      throw std::runtime_error("arm must be 'left' or 'right', got '" + s + "'");
    }

    /// 场景文件 → SceneSpec（名字部分；数值部分由 completeSceneSpec 从模型补全）
    void parseSceneSpec(const YAML::Node &sc, const std::string &root_dir, SceneSpec &spec)
    {
      spec.name = readScalar<std::string>(sc, "name", "");
      spec.description = readScalar<std::string>(sc, "description", "");
      if (!sc["model"])
        throw std::runtime_error("scene file needs 'model'");
      spec.model_path = resolvePath(root_dir, sc["model"].as<std::string>());

      const YAML::Node obj = sc["object"];
      if (!obj)
        throw std::runtime_error("scene file needs 'object'");
      spec.object.body = readScalar<std::string>(obj, "body", "");
      spec.object.geom = readScalar<std::string>(obj, "geom", "");

      for (Arm a : kArms)
      {
        const YAML::Node g = sc["grasps"][armName(a)];
        if (!g)
          throw std::runtime_error(std::string("scene file needs grasps.") + armName(a));
        GraspSpec &gs = spec.grasp[armIndex(a)];
        gs.site = readScalar<std::string>(g, "site", "");
        gs.weld = readScalar<std::string>(g, "weld", "");
        gs.finger_opening = readScalar<double>(g, "finger_opening", 0.04);
        if (gs.finger_opening < 0.0 || gs.finger_opening > 0.04)
        {
          throw std::runtime_error("finger_opening must be in [0, 0.04] m (Franka Hand)");
        }
        gs.contact_depth = readScalar<double>(g, "contact_depth", gs.contact_depth);
        gs.contact_squeeze = readScalar<double>(g, "contact_squeeze", gs.contact_squeeze);
        gs.approach_distance = readScalar<double>(g, "approach_distance", gs.approach_distance);
        if (gs.contact_squeeze < 0.0 || !(gs.approach_distance > 0.0))
          throw std::runtime_error("grasp contact_squeeze must be >= 0 and approach_distance > 0");
      }

      spec.constraints.clear();
      for (const auto &c : sc["constraints"])
      {
        ClosedChainConstraint cc;
        cc.name = readScalar<std::string>(c, "name", "");
        const std::string type = readScalar<std::string>(c, "type", "rigid");
        if (type == "rigid")
        {
          cc.type = ConstraintType::Rigid;
        }
        else if (type == "screw")
        {
          cc.type = ConstraintType::Screw;
        }
        else
        {
          throw std::runtime_error("constraint type must be rigid | screw, got '" + type + "'");
        }
        cc.body1 = readScalar<std::string>(c, "body1", "");
        cc.body2 = readScalar<std::string>(c, "body2", "");
        if (!c["relative_dof"])
          throw std::runtime_error("constraint '" + cc.name + "' needs relative_dof");
        cc.relative_dof = c["relative_dof"].as<int>();
        cc.lead = readScalar<double>(c, "lead", 0.0);
        cc.axis = readVec<3>(c, "axis", Vector3d::UnitZ()).normalized();
        cc.point = readVec<3>(c, "point", Vector3d::Zero());
        cc.hinge = readScalar<std::string>(c, "hinge", "");
        cc.slide = readScalar<std::string>(c, "slide", "");
        cc.coupling = readScalar<std::string>(c, "coupling", "");
        if (cc.type == ConstraintType::Rigid && cc.relative_dof != 0)
        {
          throw std::runtime_error("rigid constraint '" + cc.name + "' must have relative_dof 0");
        }
        if (cc.type == ConstraintType::Screw && (cc.relative_dof != 1 || cc.hinge.empty() || cc.slide.empty()))
        {
          throw std::runtime_error("screw constraint '" + cc.name + "' needs relative_dof 1, hinge, slide");
        }
        spec.constraints.push_back(cc);
      }

      spec.obstacles.clear();
      if (const YAML::Node obs = sc["obstacles"])
      {
        for (const auto &p : obs["pairs"])
        {
          ObstaclePairSpec pair;
          if (p.IsSequence() && p.size() == 2)
          {
            pair.a = p[0].as<std::string>();
            pair.b = p[1].as<std::string>();
          }
          else if (p.IsMap())
          {
            pair.a = readScalar<std::string>(p, "a", "");
            pair.b = readScalar<std::string>(p, "b", "");
            pair.safe_distance = readScalar<double>(p, "safe_distance", -1.0);
            if (pair.a.empty() || pair.b.empty() || pair.safe_distance < 0.0)
              throw std::runtime_error(
                  "obstacles.pairs map entries need a, b and non-negative safe_distance");
          }
          else
          {
            throw std::runtime_error(
                "obstacles.pairs entries must be [a, b] or {a: ..., b: ..., safe_distance: ...}");
          }
          spec.obstacles.push_back(pair);
        }
      }

      spec.waypoints.clear();
      for (const auto &w : sc["waypoints"])
      {
        Waypoint wp;
        wp.name = readScalar<std::string>(w, "name", "");
        wp.pose = poseFromYaml(w);
        if (w["screw_angle_deg"])
        {
          wp.has_screw_angle = true;
          wp.screw_angle = w["screw_angle_deg"].as<double>() * M_PI / 180.0;
        }
        spec.waypoints.push_back(wp);
      }
      if (spec.waypoints.empty())
        throw std::runtime_error("scene file needs at least one waypoint");

      spec.cameras.clear();
      for (const auto &c : sc["cameras"])
        spec.cameras.push_back(c.as<std::string>());

      for (Arm a : kArms)
      {
        spec.q_init[armIndex(a)] = readVec<7>(sc["q_init"], armName(a), Vector7d::Zero());
      }

      if (const YAML::Node s = sc["screw"])
      {
        auto &p = spec.screw;
        p.damping = readScalar<double>(s, "damping", p.damping);
        p.frictionloss = readScalar<double>(s, "frictionloss", p.frictionloss);
        p.seat_depth = readScalar<double>(s, "seat_depth", p.seat_depth);
        p.seat_stiffness = readScalar<double>(s, "seat_stiffness", p.seat_stiffness);
        p.seat_torque_max = readScalar<double>(s, "seat_torque_max", p.seat_torque_max);
      }
    }

  } // namespace

  double DisturbanceConfig::profile(double t) const
  {
    if (!enabled || t < t_start || t >= t_end)
      return 0.0;
    if (ramp <= 0.0)
      return 1.0;
    const double up = (t - t_start) / ramp;
    const double down = (t_end - t) / ramp;
    return std::clamp(std::min(up, down), 0.0, 1.0);
  }

  GravityPdConfig::GravityPdConfig()
  {
    for (auto &v : kp)
      v = Vector7d::Constant(100.0);
    for (auto &v : kd)
      v = Vector7d::Constant(10.0);
    for (auto &v : extra_torque)
      v.setZero();
  }

  std::string defaultRootDir()
  {
#ifdef DUAL_ARM_SOURCE_DIR
    return DUAL_ARM_SOURCE_DIR;
#else
    return fs::current_path().string();
#endif
  }

  std::string resolvePath(const std::string &root_dir, const std::string &path)
  {
    const fs::path p(path);
    if (p.is_absolute())
      return p.lexically_normal().string();
    return (fs::path(root_dir) / p).lexically_normal().string();
  }

  SimConfig loadConfig(const std::string &path, const std::vector<std::string> &overrides,
                       const std::string &scene, const std::string &root_dir)
  {
    SimConfig cfg;
    cfg.root_dir = root_dir;
    cfg.config_path = resolvePath(root_dir, path);

    YAML::Node root;
    try
    {
      root = YAML::LoadFile(cfg.config_path);
    }
    catch (const std::exception &e)
    {
      throw std::runtime_error("cannot read config '" + cfg.config_path + "': " + e.what());
    }

    // 场景选择：函数参数 > 命令行 --set scene=... > default.yaml
    std::string scene_name = readScalar<std::string>(root, "scene", "lift");
    for (const auto &o : overrides)
    {
      if (o.rfind("scene=", 0) == 0)
        scene_name = o.substr(6);
    }
    if (!scene.empty())
      scene_name = scene;
    root["scene"] = scene_name;
    cfg.scene_path = resolvePath(
        root_dir, readScalar<std::string>(root, "scene_dir", "config/scenes") + "/" + scene_name + ".yaml");
    YAML::Node scene_yaml;
    try
    {
      scene_yaml = YAML::LoadFile(cfg.scene_path);
    }
    catch (const std::exception &e)
    {
      throw std::runtime_error("cannot read scene '" + cfg.scene_path + "': " + e.what());
    }
    deepMerge(root, scene_yaml["overrides"]);
    for (const auto &o : overrides)
      applyOverride(root, o);

    try
    {
      parseSceneSpec(scene_yaml, root_dir, cfg.scene);

      const YAML::Node sim = root["simulation"];
      cfg.timestep = readScalar<double>(sim, "timestep", cfg.timestep);
      cfg.duration = readScalar<double>(sim, "duration", cfg.duration);
      cfg.realtime = readScalar<bool>(sim, "realtime", cfg.realtime);
      cfg.contacts = readScalar<bool>(sim, "contacts", cfg.contacts);
      cfg.contact_grasp = readScalar<bool>(sim, "contact_grasp", cfg.contact_grasp);
      cfg.contact_elliptic_cone = readScalar<bool>(sim, "contact_elliptic_cone", cfg.contact_elliptic_cone);
      cfg.contact_noslip_iterations = readScalar<int>(sim, "contact_noslip_iterations", cfg.contact_noslip_iterations);
      if (cfg.contact_noslip_iterations < 0)
        throw std::runtime_error("simulation.contact_noslip_iterations must be >= 0");
      if (cfg.contact_grasp && !cfg.contacts)
        throw std::runtime_error("simulation.contact_grasp requires simulation.contacts=true");
      if (cfg.contact_grasp && cfg.scene.name != "slot" && cfg.scene.name != "slot_avoid" &&
          cfg.scene.name != "slot_gate" && cfg.scene.name != "assembly")
        throw std::runtime_error("simulation.contact_grasp currently supports only slot, slot_avoid, slot_gate and assembly");

      const YAML::Node layout = root["layout"];
      if (!layout)
        throw std::runtime_error("config needs 'layout' (base poses)");
      for (Arm a : kArms)
      {
        const YAML::Node an = layout[armName(a)];
        if (!an)
          throw std::runtime_error(std::string("config needs layout.") + armName(a));
        auto &b = cfg.base[armIndex(a)];
        b.pos = readVec<3>(an, "base_pos", b.pos);
        b.rpy_deg = readVec<3>(an, "base_rpy_deg", b.rpy_deg);
      }
      for (Arm a : kArms)
      {
        const auto &b = cfg.base[armIndex(a)];
        cfg.scene.base[armIndex(a)].p = b.pos;
        cfg.scene.base[armIndex(a)].q = quatFromRpyDeg(b.rpy_deg);
      }

      const YAML::Node ce = root["calibration_error"];
      cfg.calibration_error.enabled = readScalar<bool>(ce, "enabled", false);
      cfg.calibration_error.right_base_offset_xyz = readVec<3>(ce, "right_base_offset_xyz", Vector3d::Zero());
      cfg.calibration_error.right_base_offset_rpy_deg =
          readVec<3>(ce, "right_base_offset_rpy_deg", Vector3d::Zero());

      const YAML::Node weld = root["weld"];
      const std::string mode = readScalar<std::string>(weld, "init_mode", "current");
      if (mode == "current")
      {
        cfg.weld.init_mode = WeldConfig::InitMode::Current;
      }
      else if (mode == "nominal")
      {
        cfg.weld.init_mode = WeldConfig::InitMode::Nominal;
      }
      else
      {
        throw std::runtime_error("weld.init_mode must be 'current' or 'nominal'");
      }
      cfg.weld.nominal_ramp_time =
          std::max(0.0, readScalar<double>(weld, "nominal_ramp_time", cfg.weld.nominal_ramp_time));
      cfg.weld.solref = readArray<2>(weld, "solref", cfg.weld.solref);
      cfg.weld.solimp = readArray<5>(weld, "solimp", cfg.weld.solimp);

      if (const YAML::Node dist = root["disturbances"])
      {
        for (const auto &dn : dist)
        {
          DisturbanceConfig d;
          d.enabled = readScalar<bool>(dn, "enabled", true);
          d.body = readScalar<std::string>(dn, "body", "");
          const std::string frame = readScalar<std::string>(dn, "frame", "world");
          if (frame == "world")
          {
            d.frame = DisturbanceConfig::Frame::World;
          }
          else if (frame == "body")
          {
            d.frame = DisturbanceConfig::Frame::Body;
          }
          else
          {
            throw std::runtime_error("disturbance frame must be world | body");
          }
          d.t_start = readScalar<double>(dn, "t_start", 0.0);
          d.t_end = readScalar<double>(dn, "t_end", 0.0);
          d.ramp = std::max(0.0, readScalar<double>(dn, "ramp", 0.0));
          d.force = readVec<3>(dn, "force", Vector3d::Zero());
          d.torque = readVec<3>(dn, "torque", Vector3d::Zero());
          cfg.disturbances.push_back(d);
        }
      }

      const YAML::Node tr = root["object_trajectory"];
      const std::string ttype = readScalar<std::string>(tr, "type", "waypoints");
      auto &ot = cfg.object_trajectory;
      if (ttype == "waypoints")
      {
        ot.type = ObjectTrajectoryConfig::Type::Waypoints;
      }
      else if (ttype == "hold")
      {
        ot.type = ObjectTrajectoryConfig::Type::Hold;
      }
      else if (ttype == "min_jerk")
      {
        ot.type = ObjectTrajectoryConfig::Type::MinJerk;
      }
      else if (ttype == "sine")
      {
        ot.type = ObjectTrajectoryConfig::Type::Sine;
      }
      else
      {
        throw std::runtime_error("object_trajectory.type must be waypoints | hold | min_jerk | sine");
      }
      if (const YAML::Node w = tr["waypoints"])
      {
        auto &p = ot.waypoints;
        p.t_start = readScalar<double>(w, "t_start", p.t_start);
        p.v_max = readScalar<double>(w, "v_max", p.v_max);
        p.a_max = readScalar<double>(w, "a_max", p.a_max);
        p.w_max = readScalar<double>(w, "w_max", p.w_max);
        p.alpha_max = readScalar<double>(w, "alpha_max", p.alpha_max);
        p.dwell = readScalar<double>(w, "dwell", p.dwell);
        if (p.v_max <= 0 || p.a_max <= 0 || p.w_max <= 0 || p.alpha_max <= 0)
        {
          throw std::runtime_error("object_trajectory.waypoints limits must be > 0");
        }
      }
      if (const YAML::Node mj = tr["min_jerk"])
      {
        ot.min_jerk.t_start = readScalar<double>(mj, "t_start", ot.min_jerk.t_start);
        ot.min_jerk.t_end = readScalar<double>(mj, "t_end", ot.min_jerk.t_end);
        ot.min_jerk.offset_xyz = readVec<3>(mj, "offset_xyz", ot.min_jerk.offset_xyz);
        ot.min_jerk.offset_rpy_deg = readVec<3>(mj, "offset_rpy_deg", ot.min_jerk.offset_rpy_deg);
      }
      if (const YAML::Node sn = tr["sine"])
      {
        ot.sine.t_start = readScalar<double>(sn, "t_start", ot.sine.t_start);
        ot.sine.frequency = readScalar<double>(sn, "frequency", ot.sine.frequency);
        ot.sine.amplitude_xyz = readVec<3>(sn, "amplitude_xyz", ot.sine.amplitude_xyz);
        ot.sine.axis = readVec<3>(sn, "axis", ot.sine.axis);
        ot.sine.amplitude_deg = readScalar<double>(sn, "amplitude_deg", ot.sine.amplitude_deg);
      }

      cfg.sensors.fix_weld_torque = readScalar<bool>(root["sensors"], "fix_weld_torque", true);
      cfg.collision.margin = readScalar<double>(root["collision"], "margin", cfg.collision.margin);
      cfg.collision.max_pairs = readScalar<int>(root["collision"], "max_pairs", cfg.collision.max_pairs);
      cfg.collision.threads = readScalar<int>(root["collision"], "threads", cfg.collision.threads);
      if (const YAML::Node pl = root["planner"])
      {
        auto &c = cfg.planner;
        c.enabled = readScalar<bool>(pl, "enabled", c.enabled);
        c.knot_dt = readScalar<double>(pl, "knot_dt", c.knot_dt);
        c.pin_start = readScalar<double>(pl, "pin_start", c.pin_start);
        c.pin_end = readScalar<double>(pl, "pin_end", c.pin_end);
        c.safety_margin = readScalar<double>(pl, "safety_margin", c.safety_margin);
        c.activation_distance = readScalar<double>(pl, "activation_distance", c.activation_distance);
        c.accel_weight = readScalar<double>(pl, "accel_weight", c.accel_weight);
        c.offset_weight = readScalar<double>(pl, "offset_weight", c.offset_weight);
        c.max_offset = readScalar<double>(pl, "max_offset", c.max_offset);
        c.max_iterations = readScalar<int>(pl, "max_iterations", c.max_iterations);
        c.trust_region = readScalar<double>(pl, "trust_region", c.trust_region);
        c.min_trust_region = readScalar<double>(pl, "min_trust_region", c.min_trust_region);
        c.penalty = readScalar<double>(pl, "penalty", c.penalty);
        c.max_penalty = readScalar<double>(pl, "max_penalty", c.max_penalty);
        c.max_speed = readScalar<double>(pl, "max_speed", c.max_speed);
        c.max_accel = readScalar<double>(pl, "max_accel", c.max_accel);
        c.multi_start = readScalar<bool>(pl, "multi_start", c.multi_start);
        c.initial_offset = readScalar<double>(pl, "initial_offset", c.initial_offset);
        c.initial_ramp = readScalar<double>(pl, "initial_ramp", c.initial_ramp);
        const std::string formulation = readScalar<std::string>(pl, "formulation", "lateral");
        if (formulation == "lateral")
          c.formulation = PlannerConfig::Formulation::Lateral;
        else if (formulation == "full")
          c.formulation = PlannerConfig::Formulation::Full;
        else
          throw std::runtime_error("planner.formulation must be lateral or full");
        c.rotation_axes = readVec<3>(pl, "rotation_axes", c.rotation_axes);
        c.max_rotation = readScalar<double>(pl, "max_rotation", c.max_rotation);
        c.rotation_length = readScalar<double>(pl, "rotation_length", c.rotation_length);
        c.rotation_offset_weight = readScalar<double>(pl, "rotation_offset_weight", c.rotation_offset_weight);
        c.time_weight = readScalar<double>(pl, "time_weight", c.time_weight);
        c.time_smooth_weight = readScalar<double>(pl, "time_smooth_weight", c.time_smooth_weight);
        c.min_dt_ratio = readScalar<double>(pl, "min_dt_ratio", c.min_dt_ratio);
        c.max_dt_ratio = readScalar<double>(pl, "max_dt_ratio", c.max_dt_ratio);
        c.max_angular_speed = readScalar<double>(pl, "max_angular_speed", c.max_angular_speed);
        c.joint_margin = readScalar<double>(pl, "joint_margin", c.joint_margin);
        c.joint_activation = readScalar<double>(pl, "joint_activation", c.joint_activation);
        c.full_max_iterations = readScalar<int>(pl, "full_max_iterations", c.full_max_iterations);
        c.model_error_body = readScalar<std::string>(pl, "model_error_body", c.model_error_body);
        c.model_error_offset = readVec<3>(pl, "model_error_offset", c.model_error_offset);
        if (!(c.max_rotation > 0.0) || !(c.rotation_length > 0.0) || c.rotation_offset_weight < 0.0 ||
            c.time_weight < 0.0 || c.time_smooth_weight < 0.0 || !(c.min_dt_ratio > 0.0) ||
            !(c.max_dt_ratio > c.min_dt_ratio) || !(c.max_angular_speed > 0.0) || c.joint_margin < 0.0 ||
            !(c.joint_activation > c.joint_margin) || c.full_max_iterations < 1)
          throw std::runtime_error("planner (full formulation) parameters are invalid");
        if (!(c.knot_dt > 0.0) || c.pin_start < 0.0 || c.pin_end < 0.0 || c.safety_margin < 0.0 ||
            !(c.activation_distance > 0.0) || !(c.accel_weight > 0.0) || c.offset_weight < 0.0 ||
            !(c.max_offset > 0.0) || c.max_iterations < 1 || !(c.trust_region > 0.0) ||
            !(c.min_trust_region > 0.0) || !(c.penalty > 0.0) || c.max_penalty < c.penalty ||
            !(c.max_speed > 0.0) || !(c.max_accel > 0.0) || c.initial_offset < 0.0 || !(c.initial_ramp > 0.0))
          throw std::runtime_error("planner parameters are invalid");
      }

      const YAML::Node ctrl = root["controller"];
      cfg.controller = readScalar<std::string>(ctrl, "type", cfg.controller);
      if (const YAML::Node pd = ctrl["gravity_pd"])
      {
        const Vector7d kp = readVec<7>(pd, "kp", cfg.gravity_pd.kp[0]);
        const Vector7d kd = readVec<7>(pd, "kd", cfg.gravity_pd.kd[0]);
        for (Arm a : kArms)
        {
          const int i = armIndex(a);
          const YAML::Node per = pd[armName(a)]; // 可选的按臂覆盖
          cfg.gravity_pd.kp[i] = per ? readVec<7>(per, "kp", kp) : kp;
          cfg.gravity_pd.kd[i] = per ? readVec<7>(per, "kd", kd) : kd;
          cfg.gravity_pd.extra_torque[i] = per ? readVec<7>(per, "extra_torque", Vector7d::Zero()) : Vector7d::Zero();
        }
      }
      if (const YAML::Node ci = ctrl["cartesian_impedance"])
      {
        auto &c = cfg.cartesian_impedance;

        c.stiffness =
            readVec<6>(ci, "stiffness", c.stiffness);

        c.damping =
            readVec<6>(ci, "damping", c.damping);

        c.load_share_left =
            readScalar<double>(
                ci,
                "load_share_left",
                c.load_share_left);

        if ((c.stiffness.array() < 0.0).any())
        {
          throw std::runtime_error(
              "cartesian_impedance.stiffness must be non-negative");
        }

        if ((c.damping.array() < 0.0).any())
        {
          throw std::runtime_error(
              "cartesian_impedance.damping must be non-negative");
        }

        if (c.load_share_left <= 0.0 ||
            c.load_share_left >= 1.0)
        {
          throw std::runtime_error(
              "cartesian_impedance.load_share_left must be in (0, 1)");
        }
      }
      if (const YAML::Node ga = ctrl["grasp_alignment"])
      {
        auto &c = cfg.grasp_alignment;
        c.approach_time = readScalar<double>(ga, "approach_time", c.approach_time);
        c.stiffness = readVec<6>(ga, "stiffness", c.stiffness);
        c.damping = readVec<6>(ga, "damping", c.damping);
        c.nullspace_kp = readScalar<double>(ga, "nullspace_kp", c.nullspace_kp);
        c.nullspace_kd = readScalar<double>(ga, "nullspace_kd", c.nullspace_kd);
        c.position_tolerance = readScalar<double>(ga, "position_tolerance", c.position_tolerance);
        c.orientation_tolerance = readScalar<double>(ga, "orientation_tolerance", c.orientation_tolerance);
        c.speed_tolerance = readScalar<double>(ga, "speed_tolerance", c.speed_tolerance);
        c.align_hold_time = readScalar<double>(ga, "align_hold_time", c.align_hold_time);
        c.close_time = readScalar<double>(ga, "close_time", c.close_time);
        c.contact_force_min = readScalar<double>(ga, "contact_force_min", c.contact_force_min);
        c.settle_time = readScalar<double>(ga, "settle_time", c.settle_time);
        c.timeout = readScalar<double>(ga, "timeout", c.timeout);
        if (!(c.approach_time > 0.0) || !(c.close_time > 0.0) || !(c.timeout > 0.0) ||
            (c.stiffness.array() < 0.0).any() || (c.damping.array() < 0.0).any() ||
            c.position_tolerance <= 0.0 || c.orientation_tolerance <= 0.0 || c.speed_tolerance <= 0.0 ||
            c.align_hold_time < 0.0 || c.settle_time < 0.0 || c.contact_force_min < 0.0)
          throw std::runtime_error("controller.grasp_alignment parameters are invalid");
      }
      if (const YAML::Node cp = ctrl["coop"])
      {
        auto &c = cfg.coop;
        c.object_stiffness = readVec<6>(cp, "object_stiffness", c.object_stiffness);
        c.object_damping = readVec<6>(cp, "object_damping", c.object_damping);
        c.internal_wrench_des = readVec<6>(cp, "internal_wrench_des", c.internal_wrench_des);
        c.internal_force_gain = readScalar<double>(cp, "internal_force_gain", c.internal_force_gain);
        c.internal_wrench_ramp_time =
            readScalar<double>(
                cp,
                "internal_wrench_ramp_time",
                c.internal_wrench_ramp_time);
        c.load_share_left = readScalar<double>(cp, "load_share_left", c.load_share_left);
        c.nullspace_kp = readScalar<double>(cp, "nullspace_kp", c.nullspace_kp);
        c.nullspace_kd = readScalar<double>(cp, "nullspace_kd", c.nullspace_kd);
        c.contact_torsion_weight =
            readScalar<double>(cp, "contact_torsion_weight", c.contact_torsion_weight);
        if (!(c.contact_torsion_weight >= 1.0))
          throw std::runtime_error("coop.contact_torsion_weight must be >= 1");
        if (c.internal_force_gain < 0.0)
        {
          throw std::runtime_error(
              "coop.internal_force_gain must be non-negative");
        }
        if (c.internal_wrench_ramp_time < 0.0)
        {
          throw std::runtime_error(
              "coop.internal_wrench_ramp_time "
              "must be non-negative");
        }
      }

      if (const YAML::Node ac = ctrl["asym_coop"])
      {
        auto &c = cfg.asym_coop;
        c.holding_arm = armFromString(readScalar<std::string>(ac, "holding_arm", "left"));
        c.holding_stiffness = readVec<6>(ac, "holding_stiffness", c.holding_stiffness);
        c.holding_damping = readVec<6>(ac, "holding_damping", c.holding_damping);
        c.task_gain = readScalar<double>(ac, "task_gain", c.task_gain);
        c.task_damping =
            readScalar<double>(
                ac,
                "task_damping",
                c.task_damping);

        c.task_torque_limit =
            readScalar<double>(
                ac,
                "task_torque_limit",
                c.task_torque_limit);
        c.working_constraint_stiffness =
            readVec<6>(
                ac,
                "working_constraint_stiffness",
                c.working_constraint_stiffness);

        c.working_constraint_damping =
            readVec<6>(
                ac,
                "working_constraint_damping",
                c.working_constraint_damping);

        c.axial_preload_force =
            readScalar<double>(
                ac,
                "axial_preload_force",
                c.axial_preload_force);

        c.axial_preload_gain =
            readScalar<double>(
                ac,
                "axial_preload_gain",
                c.axial_preload_gain);

        c.axial_preload_limit =
            readScalar<double>(
                ac,
                "axial_preload_limit",
                c.axial_preload_limit);

        c.axial_preload_ramp_time =
            readScalar<double>(
                ac,
                "axial_preload_ramp_time",
                c.axial_preload_ramp_time);

        c.preload_ready_tolerance =
            readScalar<double>(ac, "preload_ready_tolerance", c.preload_ready_tolerance);
        c.preload_ready_hold_time =
            readScalar<double>(ac, "preload_ready_hold_time", c.preload_ready_hold_time);
        c.wrench_filter_time_constant =
            readScalar<double>(ac, "wrench_filter_time_constant", c.wrench_filter_time_constant);
        c.seat_torque_threshold =
            readScalar<double>(ac, "seat_torque_threshold", c.seat_torque_threshold);
        c.seat_detect_hold_time =
            readScalar<double>(ac, "seat_detect_hold_time", c.seat_detect_hold_time);
        c.tightening_torque =
            readScalar<double>(ac, "tightening_torque", c.tightening_torque);
        c.tightening_admittance =
            readScalar<double>(ac, "tightening_admittance", c.tightening_admittance);
        c.tightening_max_rate =
            readScalar<double>(ac, "tightening_max_rate", c.tightening_max_rate);
        c.tightening_torque_limit =
            readScalar<double>(ac, "tightening_torque_limit", c.tightening_torque_limit);
        c.tightening_torque_ramp_time =
            readScalar<double>(ac, "tightening_torque_ramp_time", c.tightening_torque_ramp_time);
        c.completion_torque_tolerance =
            readScalar<double>(ac, "completion_torque_tolerance", c.completion_torque_tolerance);
        c.completion_speed_threshold =
            readScalar<double>(ac, "completion_speed_threshold", c.completion_speed_threshold);
        c.completion_preload_tolerance =
            readScalar<double>(ac, "completion_preload_tolerance", c.completion_preload_tolerance);
        c.completion_hold_time =
            readScalar<double>(ac, "completion_hold_time", c.completion_hold_time);

        if ((c.holding_stiffness.array() < 0.0).any() ||
            (c.holding_damping.array() < 0.0).any() ||
            (c.working_constraint_stiffness.array() < 0.0).any() ||
            (c.working_constraint_damping.array() < 0.0).any())
        {
          throw std::runtime_error(
              "asym_coop impedance gains must be non-negative");
        }

        if (c.task_gain < 0.0 ||
            c.task_damping < 0.0 ||
            c.task_torque_limit < 0.0)
        {
          throw std::runtime_error(
              "asym_coop task gains and torque limit "
              "must be non-negative");
        }

        if (c.axial_preload_force < 0.0 ||
            c.axial_preload_gain < 0.0 ||
            c.axial_preload_limit < c.axial_preload_force ||
            c.axial_preload_ramp_time < 0.0 ||
            c.preload_ready_tolerance < 0.0 ||
            c.preload_ready_hold_time < 0.0 ||
            c.wrench_filter_time_constant < 0.0 ||
            c.seat_torque_threshold < 0.0 ||
            c.seat_detect_hold_time < 0.0 ||
            c.tightening_torque < c.seat_torque_threshold ||
            c.tightening_admittance < 0.0 ||
            c.tightening_max_rate < 0.0 ||
            c.tightening_torque_limit < c.tightening_torque ||
            c.tightening_torque_ramp_time < 0.0 ||
            c.completion_torque_tolerance < 0.0 ||
            c.completion_speed_threshold < 0.0 ||
            c.completion_preload_tolerance < 0.0 ||
            c.completion_hold_time < 0.0)
        {
          throw std::runtime_error(
              "asym_coop axial preload parameters are invalid");
        }
      }

      if (const YAML::Node tq = ctrl["torque_qp"])
      {
        auto &c = cfg.torque_qp;
        c.tracking_weight =
            readScalar<double>(tq, "tracking_weight", c.tracking_weight);
        c.smoothing_weight =
            readScalar<double>(tq, "smoothing_weight", c.smoothing_weight);
        c.torque_margin =
            readScalar<double>(tq, "torque_margin", c.torque_margin);
        c.joint_position_margin =
            readScalar<double>(tq, "joint_position_margin", c.joint_position_margin);
        c.joint_prediction_horizon =
            readScalar<double>(tq, "joint_prediction_horizon", c.joint_prediction_horizon);
        c.joint_velocity_limit =
            readVec<7>(tq, "joint_velocity_limit", c.joint_velocity_limit);
        c.joint_acceleration_limit =
            readVec<7>(tq, "joint_acceleration_limit", c.joint_acceleration_limit);
        c.closed_chain_velocity_damping =
            readScalar<double>(
                tq,
                "closed_chain_velocity_damping",
                c.closed_chain_velocity_damping);
        c.constraint_wrench_regularization =
            readScalar<double>(
                tq,
                "constraint_wrench_regularization",
                c.constraint_wrench_regularization);
        c.collision_avoidance_enabled =
            readScalar<bool>(
                tq,
                "collision_avoidance_enabled",
                c.collision_avoidance_enabled);
        c.collision_safe_distance =
            readScalar<double>(tq, "collision_safe_distance", c.collision_safe_distance);
        c.collision_influence_distance =
            readScalar<double>(tq, "collision_influence_distance", c.collision_influence_distance);
        c.collision_max_approach_speed =
            readScalar<double>(tq, "collision_max_approach_speed", c.collision_max_approach_speed);
        c.collision_slack_weight =
            readScalar<double>(tq, "collision_slack_weight", c.collision_slack_weight);
        c.collision_max_constraints =
            readScalar<int>(tq, "collision_max_constraints", c.collision_max_constraints);
        c.collision_threads =
            readScalar<int>(tq, "collision_threads", c.collision_threads);
        c.admm_rho =
            readScalar<double>(tq, "admm_rho", c.admm_rho);
        c.admm_max_iterations =
            readScalar<int>(tq, "admm_max_iterations", c.admm_max_iterations);
        c.admm_tolerance =
            readScalar<double>(tq, "admm_tolerance", c.admm_tolerance);
        if (const YAML::Node rg = tq["reference_governor"])
        {
          auto &g = c.reference_governor;
          g.enabled = readScalar<bool>(rg, "enabled", g.enabled);
          g.obstacle_group =
              readScalar<std::string>(rg, "obstacle_group", g.obstacle_group);
          g.trigger_distance =
              readScalar<double>(rg, "trigger_distance", g.trigger_distance);
          g.release_distance =
              readScalar<double>(rg, "release_distance", g.release_distance);
          g.avoidance_offset =
              readScalar<double>(rg, "avoidance_offset", g.avoidance_offset);
          g.offset_speed =
              readScalar<double>(rg, "offset_speed", g.offset_speed);
          g.preferred_direction =
              readVec<3>(rg, "preferred_direction", g.preferred_direction);
          const std::string governor_mode = readScalar<std::string>(rg, "mode", "state_machine");
          if (governor_mode == "state_machine")
            g.mode = TorqueQpConfig::ReferenceGovernorConfig::Mode::StateMachine;
          else if (governor_mode == "cbf")
            g.mode = TorqueQpConfig::ReferenceGovernorConfig::Mode::Cbf;
          else
            throw std::runtime_error("torque_qp.reference_governor.mode must be state_machine or cbf");
          if (const YAML::Node cb = rg["cbf"])
          {
            auto &cb_cfg = g.cbf;
            cb_cfg.safe_distance = readScalar<double>(cb, "safe_distance", cb_cfg.safe_distance);
            cb_cfg.alpha = readScalar<double>(cb, "alpha", cb_cfg.alpha);
            cb_cfg.offset_max = readScalar<double>(cb, "offset_max", cb_cfg.offset_max);
            cb_cfg.offset_speed_max = readScalar<double>(cb, "offset_speed_max", cb_cfg.offset_speed_max);
            cb_cfg.offset_accel_max = readScalar<double>(cb, "offset_accel_max", cb_cfg.offset_accel_max);
            cb_cfg.time_rate_accel_max = readScalar<double>(cb, "time_rate_accel_max", cb_cfg.time_rate_accel_max);
            cb_cfg.return_gain = readScalar<double>(cb, "return_gain", cb_cfg.return_gain);
            cb_cfg.offset_weight = readScalar<double>(cb, "offset_weight", cb_cfg.offset_weight);
            cb_cfg.time_rate_weight = readScalar<double>(cb, "time_rate_weight", cb_cfg.time_rate_weight);
            cb_cfg.escape_direction = readVec<3>(cb, "escape_direction", cb_cfg.escape_direction);
            cb_cfg.escape_gain = readScalar<double>(cb, "escape_gain", cb_cfg.escape_gain);
            cb_cfg.escape_activation = readScalar<double>(cb, "escape_activation", cb_cfg.escape_activation);
            if (cb_cfg.safe_distance < 0.0 || !(cb_cfg.alpha > 0.0) || !(cb_cfg.offset_max > 0.0) ||
                !(cb_cfg.offset_speed_max > 0.0) || !(cb_cfg.offset_accel_max > 0.0) || !(cb_cfg.time_rate_accel_max > 0.0) ||
                cb_cfg.return_gain < 0.0 || !(cb_cfg.offset_weight > 0.0) || !(cb_cfg.time_rate_weight > 0.0) ||
                cb_cfg.escape_gain < 0.0 || cb_cfg.escape_activation < 0.0 || cb_cfg.escape_direction.norm() < 1e-9)
              throw std::runtime_error("torque_qp.reference_governor.cbf parameters are invalid");
            cb_cfg.escape_direction.normalize();
          }
          if (g.enabled &&
              (g.obstacle_group.empty() ||
               !(g.trigger_distance > 0.0) ||
               !(g.release_distance > 0.0) ||
               !(g.avoidance_offset > 0.0) ||
               !(g.offset_speed > 0.0) ||
               g.preferred_direction.norm() < 1e-9))
          {
            throw std::runtime_error("torque_qp.reference_governor parameters are invalid");
          }
          g.preferred_direction.normalize();
        }

        if (!(c.tracking_weight > 0.0) ||
            c.smoothing_weight < 0.0 ||
            c.torque_margin < 0.0 ||
            c.joint_position_margin < 0.0 ||
            !(c.joint_prediction_horizon > 0.0) ||
            (c.joint_velocity_limit.array() <= 0.0).any() ||
            (c.joint_acceleration_limit.array() <= 0.0).any() ||
            c.closed_chain_velocity_damping < 0.0 ||
            !(c.constraint_wrench_regularization > 0.0) ||
            c.collision_safe_distance < 0.0 ||
            c.collision_influence_distance <= c.collision_safe_distance ||
            !(c.collision_max_approach_speed > 0.0) ||
            !(c.collision_slack_weight > 0.0) ||
            c.collision_max_constraints < 0 ||
            c.collision_max_constraints > 8 ||
            c.collision_threads < 0 ||
            !(c.admm_rho > 0.0) ||
            c.admm_max_iterations <= 0 ||
            !(c.admm_tolerance > 0.0))
        {
          throw std::runtime_error(
              "torque_qp parameters are invalid");
        }
      }

      const YAML::Node lg = root["log"];
      cfg.log.enabled = readScalar<bool>(lg, "enabled", cfg.log.enabled);
      cfg.log.dir = resolvePath(root_dir, readScalar<std::string>(lg, "dir", cfg.log.dir));
      cfg.log.decimation = std::max(1, readScalar<int>(lg, "decimation", cfg.log.decimation));
    }
    catch (const YAML::Exception &e)
    {
      throw std::runtime_error("config '" + cfg.config_path + "' / scene '" + cfg.scene_path + "': " + e.what());
    }

    if (cfg.timestep <= 0.0)
      throw std::runtime_error("simulation.timestep must be > 0");
    if (cfg.weld.solref[0] < 2.0 * cfg.timestep)
    {
      std::fprintf(stderr,
                   "[config] warning: weld.solref timeconst %.4g < 2*timestep; MuJoCo will clamp it\n",
                   cfg.weld.solref[0]);
    }

    cfg.scene.contact_grasp = cfg.contact_grasp;

    YAML::Node effective = YAML::Clone(root);
    effective["scene_spec"] = YAML::Clone(scene_yaml);
    YAML::Emitter out;
    out << effective;
    cfg.effective_yaml = out.c_str();
    return cfg;
  }

  Pose nominalBasePose(const SimConfig &cfg, Arm arm)
  {
    const auto &b = cfg.base[armIndex(arm)];
    Pose T;
    T.p = b.pos;
    T.q = quatFromRpyDeg(b.rpy_deg);
    return T;
  }

  Pose plantBasePose(const SimConfig &cfg, Arm arm)
  {
    Pose T = nominalBasePose(cfg, arm);
    const auto &ce = cfg.calibration_error;
    if (arm == Arm::Right && ce.enabled)
    {
      T.p += ce.right_base_offset_xyz;
      T.q = (quatFromRpyDeg(ce.right_base_offset_rpy_deg) * T.q).normalized();
    }
    return T;
  }

} // namespace dual_arm
