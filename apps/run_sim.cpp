/**
 * run_sim —— 双 Panda 协同操作仿真主程序（场景由 --scene 选择，见 config/scenes/）
 *
 * 用法：
 *   ./build/run_sim [--scene lift|slot|slot_avoid|assembly|assembly_simple]
 *                   [--controller gravity_pd|coop|qp_coop|asym_coop|qp_asym_coop]
 *                   [--headless] [--duration SEC] [--config config/default.yaml]
 *                   [--calib-error | --no-calib-error] [--set key.path=value ...] [--no-log] [--no-realtime]
 *                   [--camera cam_iso|cam_front] [--screenshot out.png] [--record out.mp4 [--record-fps 30]]
 *
 * 每个控制周期（1 kHz）：
 *   state = env.state()                          q, dq, 末端位姿(t)；F/T、weld 力为上一周期测量
 *   (τ_l, τ_r) = controller.compute(state, t)    控制器内部调用 RobotModel::update（名义模型）
 *   env.step(τ_l, τ_r)                           饱和、施加扰动 / 螺钉阻力矩、mj_step2 + mj_step1
 *   logger.write(env.lastStep(), monitor)        一行 CSV（所有字段对齐到 t；场景列来自 SceneMonitor）
 */
#include "dual_arm/asym_coop_controller.hpp"
#include "dual_arm/baseline_controllers.hpp"
#include "dual_arm/config.hpp"
#include "dual_arm/coop_controller.hpp"
#include "dual_arm/coop_kinematics.hpp"
#include "dual_arm/logger.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/qp_coop_controller.hpp"
#include "dual_arm/qp_asym_coop_controller.hpp"
#include "dual_arm/scene_monitor.hpp"
#include "dual_arm/sim_env.hpp"
#ifdef DUAL_ARM_HAS_VIEWER
#include "dual_arm/viewer.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <memory>
#include <string>
#include <vector>

using namespace dual_arm;
using Clock = std::chrono::steady_clock;

namespace
{

  struct Args
  {
    std::string config = "config/default.yaml";
    std::string scene;
    std::vector<std::string> overrides;
    bool headless = false;
    bool log = true;
    bool realtime = true;
    std::string screenshot;
    std::string record;
    double record_fps = 30.0;
    std::string camera;
    std::string cmdline;
  };

  void printUsage()
  {
    std::printf(
        "Usage: run_sim [options]\n"
        "  --scene NAME           场景：lift | slot | slot_avoid | assembly | assembly_simple（覆盖 default.yaml 中的 scene）\n"
        "  --config PATH          公共配置文件（默认 config/default.yaml，相对工程根目录）\n"
        "  --controller NAME      gravity_pd | coop | qp_coop | asym_coop | qp_asym_coop（覆盖 controller.type）\n"
        "  --headless             无界面，全速运行\n"
        "  --duration SEC         仿真时长（覆盖 simulation.duration）\n"
        "  --calib-error          打开右臂基座标定误差（calibration_error.enabled=true）\n"
        "  --no-calib-error       关闭标定误差\n"
        "  --set key.path=value   覆盖任意配置项（可重复），值按 YAML 解析\n"
        "  --no-log               不写日志\n"
        "  --no-realtime          可视化模式下不按实时速率限制\n"
        "  --camera NAME          使用场景中的固定相机：cam_iso | cam_front（可视化默认自由相机）\n"
        "  --screenshot PATH      隐藏窗口运行 --duration 秒（可为 0）后保存截图并退出；\n"
        "                         .png 存 PNG，否则存 PPM；默认相机 cam_iso\n"
        "  --record PATH          隐藏窗口运行 --duration 秒并录制视频（ffmpeg，H.264）；默认相机 cam_iso\n"
        "  --record-fps FPS       录像帧率（按仿真时间，默认 30）\n"
        "  -h, --help             显示帮助\n");
  }

  Args parseArgs(int argc, char **argv)
  {
    Args a;
    for (int i = 0; i < argc; ++i)
    {
      a.cmdline += (i ? " " : "") + std::string(argv[i]);
    }
    auto need = [&](int &i) -> std::string
    {
      if (i + 1 >= argc)
        throw std::runtime_error(std::string("missing value after ") + argv[i]);
      return argv[++i];
    };
    for (int i = 1; i < argc; ++i)
    {
      const std::string s = argv[i];
      if (s == "-h" || s == "--help")
      {
        printUsage();
        std::exit(0);
      }
      else if (s == "--config")
      {
        a.config = need(i);
      }
      else if (s == "--scene")
      {
        a.scene = need(i);
      }
      else if (s == "--controller")
      {
        a.overrides.push_back("controller.type=" + need(i));
      }
      else if (s == "--headless")
      {
        a.headless = true;
      }
      else if (s == "--duration")
      {
        a.overrides.push_back("simulation.duration=" + need(i));
      }
      else if (s == "--calib-error")
      {
        a.overrides.push_back("calibration_error.enabled=true");
      }
      else if (s == "--no-calib-error")
      {
        a.overrides.push_back("calibration_error.enabled=false");
      }
      else if (s == "--set")
      {
        a.overrides.push_back(need(i));
      }
      else if (s == "--no-log")
      {
        a.log = false;
      }
      else if (s == "--no-realtime")
      {
        a.realtime = false;
      }
      else if (s == "--camera")
      {
        a.camera = need(i);
      }
      else if (s == "--screenshot")
      {
        a.screenshot = need(i);
      }
      else if (s == "--record")
      {
        a.record = need(i);
      }
      else if (s == "--record-fps")
      {
        a.record_fps = std::stod(need(i));
      }
      else
      {
        throw std::runtime_error("unknown argument '" + s + "' (see --help)");
      }
    }
    return a;
  }

  std::unique_ptr<Controller> makeController(const SimConfig &cfg, std::shared_ptr<RobotModel> model,
                                             std::shared_ptr<const ObjectTrajectory> traj)
  {
    if (cfg.controller == "gravity_pd")
      return std::make_unique<GravityCompJointPD>(model, cfg.gravity_pd);
    if (cfg.controller == "cartesian_impedance")
    {
      return std::make_unique<
          IndependentCartesianImpedance>(
          model,
          traj,
          cfg.cartesian_impedance);
    }
    if (cfg.controller == "coop")
      return std::make_unique<CoopController>(model, traj, cfg.coop);
    if (cfg.controller == "qp_coop")
      return std::make_unique<QpCoopController>(
          model,
          traj,
          cfg.coop,
          cfg.torque_qp,
          cfg.collision,
          cfg.timestep);
    if (cfg.controller == "asym_coop")
      return std::make_unique<AsymmetricCoopController>(model, traj, cfg.asym_coop);
    if (cfg.controller == "qp_asym_coop")
      return std::make_unique<QpAsymmetricCoopController>(
          model,
          traj,
          cfg.asym_coop,
          cfg.torque_qp,
          cfg.collision,
          cfg.timestep);

    throw std::runtime_error(
        "unknown controller '" + cfg.controller +
        "' (gravity_pd | cartesian_impedance | coop | qp_coop | asym_coop | qp_asym_coop)");
  }

  /// 打印初始构型的运动学指标（基于控制器模型）：雅可比条件数、各关节到限位的裕度
  void printInitialConfiguration(const SimEnv &env, MujocoRobotModel &model)
  {
    model.update(env.state());
    std::printf("[run_sim] initial configuration (controller model, TCP = ee_site):\n");
    for (Arm a : kArms)
    {
      const Vector7d q = env.state().arm(a).q;
      const Vector7d lo = model.jointLowerLimit(a), hi = model.jointUpperLimit(a);
      const JacobianMetrics jm = jacobianMetrics(model.jacobian(a));
      const Pose T = model.eePose(a);
      std::printf("[run_sim]   %-5s q = [", armName(a));
      for (int j = 0; j < kArmDof; ++j)
        std::printf("%s%.4f", j ? " " : "", q[j]);
      std::printf("]  TCP = (%.3f, %.3f, %.3f)\n", T.p.x(), T.p.y(), T.p.z());
      std::printf("[run_sim]         cond(J) = %.2f  sigma_min(J) = %.3f  cond(J_v) = %.2f  "
                  "sigma_min(J_v) = %.3f m/rad  manipulability = %.4f\n",
                  jm.cond, jm.sigma_min, jm.cond_linear, jm.sigma_min_linear, jm.manipulability);
      std::printf("[run_sim]         margin to limit [deg / %% of range]:");
      int worst = 0;
      double worst_frac = 1.0;
      for (int j = 0; j < kArmDof; ++j)
      {
        const double m = std::min(q[j] - lo[j], hi[j] - q[j]);
        const double frac = m / (hi[j] - lo[j]);
        if (frac < worst_frac)
        {
          worst_frac = frac;
          worst = j;
        }
        std::printf(" j%d %.1f/%.0f%%", j + 1, m * 180.0 / M_PI, 100.0 * frac);
      }
      std::printf("  (min: j%d)\n", worst + 1);
    }
  }

  void printSetup(const SimConfig &cfg, const SimEnv &env, const SceneMonitor &mon)
  {
    const SceneSpec &sc = env.scene();
    std::printf("[run_sim] MuJoCo %s | scene '%s': %s\n", mj_versionString(), sc.name.c_str(), sc.description.c_str());
    std::printf("[run_sim] model %s\n", sc.model_path.c_str());
    for (Arm a : kArms)
    {
      const auto &b = cfg.base[armIndex(a)];
      std::printf("[run_sim] %-5s base (nominal) pos = [%.4f %.4f %.4f] m, rpy = [%.2f %.2f %.2f] deg\n", armName(a),
                  b.pos.x(), b.pos.y(), b.pos.z(), b.rpy_deg.x(), b.rpy_deg.y(), b.rpy_deg.z());
    }
    std::printf("[run_sim] object '%s': %.3f kg | grasps: left -> %s (%s), right -> %s (%s)\n", sc.object.body.c_str(),
                sc.object.mass, sc.grasp[0].body.c_str(), sc.grasp[0].site.c_str(), sc.grasp[1].body.c_str(),
                sc.grasp[1].site.c_str());
    std::printf("[run_sim] closed chain:");
    for (const auto &c : sc.constraints)
    {
      std::printf(" %s[%s %s-%s, rel %d]", c.name.c_str(), c.type == ConstraintType::Rigid ? "rigid" : "screw",
                  c.body1.c_str(), c.body2.c_str(), c.relative_dof);
    }
    const int nr = sc.relativeDofBetweenHands();
    std::printf("\n[run_sim] relative DOF between hands = %d -> internal-force dimension = %d\n", nr, 6 - nr);
    std::printf("[run_sim] controller = %s | dt = %.4g s | duration = %.3g s | contacts %s\n", cfg.controller.c_str(),
                env.timestep(), cfg.duration, cfg.contacts ? "on" : "off");
    const auto &ce = cfg.calibration_error;
    if (ce.enabled)
    {
      std::printf("[run_sim] calibration error ON: right base offset xyz = [%.4f %.4f %.4f] m, "
                  "rpy = [%.3f %.3f %.3f] deg\n",
                  ce.right_base_offset_xyz.x(), ce.right_base_offset_xyz.y(), ce.right_base_offset_xyz.z(),
                  ce.right_base_offset_rpy_deg.x(), ce.right_base_offset_rpy_deg.y(), ce.right_base_offset_rpy_deg.z());
    }
    else
    {
      std::printf("[run_sim] calibration error OFF\n");
    }
    std::printf("[run_sim] weld init_mode = %s, solref = [%.4g %.4g]\n",
                cfg.weld.init_mode == WeldConfig::InitMode::Current ? "current" : "nominal", cfg.weld.solref[0],
                cfg.weld.solref[1]);
    for (Arm a : kArms)
    {
      const Vector6d &e = env.initialGraspMismatch(a);
      std::printf("[run_sim] %-5s ee_site vs grasp site at t=0: |dp| = %.3f mm, |dtheta| = %.4f deg\n", armName(a),
                  1e3 * e.head<3>().norm(), e.tail<3>().norm() * 180.0 / M_PI);
    }
    for (const auto &d : cfg.disturbances)
    {
      if (!d.enabled)
        continue;
      std::printf("[run_sim] disturbance on '%s' (%s frame): t in [%.3g, %.3g) s, ramp %.3g s, F = [%g %g %g] N, "
                  "M = [%g %g %g] N·m\n",
                  d.body.empty() ? sc.object.body.c_str() : d.body.c_str(),
                  d.frame == DisturbanceConfig::Frame::World ? "world" : "body", d.t_start, d.t_end, d.ramp,
                  d.force.x(), d.force.y(), d.force.z(), d.torque.x(), d.torque.y(), d.torque.z());
    }
    if (const CollisionModel *cm = mon.collision())
    {
      std::printf("[run_sim] obstacle pairs: %d geom pairs in %d groups (margin %.2f m); distances at t=0:\n",
                  cm->numPairs(), cm->numGroups(), cm->margin());
      for (int g = 0; g < cm->numGroups(); ++g)
      {
        std::printf("[run_sim]   %-28s %7.1f mm\n", cm->groupName(g).c_str(), 1e3 * cm->groupMinDistance(g));
      }
    }
  }

  /// 运行过程中的统计量
  struct Stats
  {
    long steps = 0;
    double ctrl_sum_us = 0.0, ctrl_max_us = 0.0;
    double step_sum_us = 0.0;
    double obj_dev_max = 0.0;              ///< 物体相对初始位置的最大偏移 [m]
    double weld_force_max[2] = {0.0, 0.0}; ///< |f_weld| 最大值 [N]
    double tau_sat_steps = 0;              ///< 有关节力矩达到上限的步数
    double d_min = 1e9;                    ///< 记录到的最小障碍距离 [m]
    double governor_offset_max = 0.0;       ///< 自主避障最大参考偏移 [m]
    long governor_active_steps = 0;         ///< governor 非 Normal 的控制周期数
    int governor_events = 0;                ///< Normal -> Lift 的触发次数
    QpCoopController::GovernorPhase governor_previous_phase =
        QpCoopController::GovernorPhase::Normal;
  };

} // namespace

int main(int argc, char **argv)
{
  Args args;
  SimConfig cfg;
  try
  {
    args = parseArgs(argc, argv);
    cfg = loadConfig(args.config, args.overrides, args.scene);
  }
  catch (const std::exception &e)
  {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }

  try
  {
    const auto t_load0 = Clock::now();
    SimEnv env(cfg);
    auto model = std::make_shared<MujocoRobotModel>(cfg);
    auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose,
                                                   env.scene().waypoints);
    auto controller = makeController(cfg, model, traj);
    auto *asym_controller =
        dynamic_cast<AsymmetricCoopController *>(
            controller.get());
    auto *qp_asym_controller =
        dynamic_cast<QpAsymmetricCoopController *>(
            controller.get());
    auto *qp_coop_controller =
        dynamic_cast<QpCoopController *>(
            controller.get());
    controller->reset(env.state());
    SceneMonitor monitor(env);
    monitor.update(env.state());
    const double load_s = std::chrono::duration<double>(Clock::now() - t_load0).count();
    printSetup(cfg, env, monitor);
    printInitialConfiguration(env, *model);
    std::printf("[run_sim] models loaded in %.2f s (plant + controller model + collision model)\n", load_s);

    // ---------------- 日志 ----------------
    CsvLogger logger;
    std::string run_dir;
    if (cfg.log.enabled && args.log)
    {
      std::string tag = cfg.scene.name + "_" + cfg.controller;
      if (cfg.calibration_error.enabled)
        tag += "_calib";
      run_dir = makeRunDirectory(cfg.log.dir, tag);
      const std::time_t now = std::time(nullptr);
      char stamp[64];
      std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
      writeTextFile(run_dir + "/config.yaml",
                    "# effective configuration (after scene overrides and command-line overrides)\n# generated " +
                        std::string(stamp) + "\n# source: " + cfg.config_path + " + " + cfg.scene_path +
                        "\n# command: " + args.cmdline + "\n" + cfg.effective_yaml + "\n");
      writeTextFile(run_dir + "/run_info.txt", "command: " + args.cmdline + "\nmujoco: " + mj_versionString() +
                                                   "\nscene: " + cfg.scene.name + "\nmodel: " +
                                                   cfg.scene.model_path + "\ncontroller: " + cfg.controller + "\n");
      logger.setExtraColumns(monitor.columns());
      if (!logger.open(run_dir + "/log.csv"))
        throw std::runtime_error("cannot open " + run_dir + "/log.csv");
      std::printf("[run_sim] logging to %s/log.csv\n", run_dir.c_str());
    }

    Stats st;
    const Vector3d obj_p0 = env.state().object.pose.p;
    const Vector7d tau_lim_l = env.torqueLimit(Arm::Left);
    const Vector7d tau_lim_r = env.torqueLimit(Arm::Right);
    LogRow row;
    bool diverged = false;

    // 单个控制周期
    auto stepOnce = [&]()
    {
      const DualArmState &s = env.state();
      const auto t0 = Clock::now();
      const auto [tau_l, tau_r] = controller->compute(s, s.t);
      const auto t1 = Clock::now();
      env.step(tau_l, tau_r);
      const auto t2 = Clock::now();

      const DualArmState &rec = env.lastStep();
      const double ctrl_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      const double step_us = std::chrono::duration<double, std::micro>(t2 - t1).count();
      st.ctrl_sum_us += ctrl_us;
      st.ctrl_max_us = std::max(st.ctrl_max_us, ctrl_us);
      st.step_sum_us += step_us;
      st.obj_dev_max = std::max(st.obj_dev_max, (rec.object.pose.p - obj_p0).norm());
      for (Arm a : kArms)
      {
        double &fm = st.weld_force_max[armIndex(a)];
        fm = std::max(fm, rec.arm(a).weld_wrench.head<3>().norm());
      }
      if ((rec.arm(Arm::Left).tau.cwiseAbs() - tau_lim_l).maxCoeff() >= -1e-9 ||
          (rec.arm(Arm::Right).tau.cwiseAbs() - tau_lim_r).maxCoeff() >= -1e-9)
      {
        st.tau_sat_steps += 1;
      }
      if (qp_coop_controller)
      {
        const auto phase = qp_coop_controller->governorPhase();
        st.governor_offset_max = std::max(
            st.governor_offset_max,
            qp_coop_controller->governorOffset());
        if (phase != QpCoopController::GovernorPhase::Normal)
          ++st.governor_active_steps;
        if (phase == QpCoopController::GovernorPhase::Lift &&
            st.governor_previous_phase == QpCoopController::GovernorPhase::Normal)
          ++st.governor_events;
        st.governor_previous_phase = phase;
      }

      if (logger.isOpen() && st.steps % cfg.log.decimation == 0)
      {
        const ObjectReference ref = qp_coop_controller
            ? qp_coop_controller->governedReference()
            : traj->evaluate(rec.t);
        monitor.update(rec);
        if (monitor.collision())
          st.d_min = std::min(st.d_min, monitor.collision()->minDistance());
        row.step = &rec;
        row.object_ref = ref.pose;
        row.screw_ref = ref.screw_angle;
        row.object_error = poseError(ref.pose, rec.object.pose);
        row.internal_wrench = coop::internalWrenchForLogging(rec, *model);
        if (asym_controller)
        {
          row.assembly_phase =
              static_cast<int>(asym_controller->phase());
          row.tightening_torque_ref =
              asym_controller->tighteningTorqueReference();
          row.tightening_torque_meas =
              asym_controller->measuredTighteningTorque();
          row.preload_ref =
              asym_controller->preloadReference();
          row.preload_meas =
              asym_controller->measuredPreload();
        }
        else if (qp_asym_controller)
        {
          row.assembly_phase =
              static_cast<int>(qp_asym_controller->phase());
          row.tightening_torque_ref =
              qp_asym_controller->tighteningTorqueReference();
          row.tightening_torque_meas =
              qp_asym_controller->measuredTighteningTorque();
          row.preload_ref =
              qp_asym_controller->preloadReference();
          row.preload_meas =
              qp_asym_controller->measuredPreload();
        }
        if (qp_coop_controller)
        {
          row.governor_phase = static_cast<int>(
              qp_coop_controller->governorPhase());
          row.governor_offset = qp_coop_controller->governorOffset();
          row.governor_virtual_time =
              qp_coop_controller->governorVirtualTime();
        }
        row.ctrl_time_us = ctrl_us;
        row.step_time_us = step_us;
        row.extra = monitor.values().data();
        logger.write(row);
      }
      ++st.steps;
      if (env.diverged())
        diverged = true;
    };
    auto finished = [&]()
    { return env.time() >= cfg.duration - 0.5 * env.timestep() || diverged; };

    const auto wall0 = Clock::now();
    const bool offscreen = !args.screenshot.empty() || !args.record.empty();
    if (args.headless || offscreen)
    {
      // ---------- 无界面 / 截图 / 录像：全速运行 ----------
      if (cfg.duration <= 0.0 && !offscreen)
        throw std::runtime_error("headless mode needs duration > 0");
#ifdef DUAL_ARM_HAS_VIEWER
      std::unique_ptr<Viewer> viewer;
      std::unique_ptr<VideoRecorder> video;
      std::vector<unsigned char> frame;
      int fw = 0, fh = 0;
      if (offscreen)
      {
        Viewer::Options vo;
        vo.visible = false;
        vo.camera = args.camera.empty() ? "cam_iso" : args.camera;
        viewer = std::make_unique<Viewer>(env.model(), vo);
      }
      double next_frame_t = 0.0;
      auto grab = [&]()
      {
        viewer->renderOffscreen(env.data(), frame, &fw, &fh);
        if (!video)
          video = std::make_unique<VideoRecorder>(args.record, fw, fh, args.record_fps);
        video->write(frame);
        next_frame_t += 1.0 / args.record_fps;
      };
      if (!args.record.empty())
        grab();
      while (!finished())
      {
        stepOnce();
        if (!args.record.empty() && env.time() >= next_frame_t - 0.5 * env.timestep())
          grab();
      }
      if (video)
      {
        const int n = video->frames();
        if (video->close())
        {
          std::printf("[run_sim] video saved to %s (%d frames @ %.0f fps)\n", args.record.c_str(), n, args.record_fps);
        }
        else
        {
          std::fprintf(stderr, "[run_sim] ffmpeg failed while writing %s\n", args.record.c_str());
        }
      }
      if (!args.screenshot.empty())
      {
        if (viewer->saveScreenshot(env.data(), args.screenshot))
        {
          std::printf("[run_sim] screenshot saved to %s\n", args.screenshot.c_str());
        }
        else
        {
          std::fprintf(stderr, "[run_sim] failed to save screenshot\n");
        }
      }
#else
      if (offscreen)
        throw std::runtime_error("built without viewer: --screenshot / --record unavailable");
      while (!finished())
        stepOnce();
#endif
    }
    else
    {
#ifdef DUAL_ARM_HAS_VIEWER
      // ---------- 可视化：按实时速率推进，约 60 Hz 渲染 ----------
      Viewer::Options vo;
      vo.title = "dual_arm - " + cfg.scene.name + " - " + controller->name();
      vo.camera = args.camera;
      Viewer viewer(env.model(), vo);
      const bool realtime = cfg.realtime && args.realtime;
      auto sync_wall = Clock::now();
      double sync_sim = env.time();
      bool was_paused = false;
      char title[256], values[512];
      while (!viewer.shouldClose())
      {
        const bool done = (cfg.duration > 0.0 && finished()) || diverged;
        if (viewer.paused() || done)
        {
          was_paused = true;
        }
        else
        {
          if (was_paused)
          {
            sync_wall = Clock::now();
            sync_sim = env.time();
            was_paused = false;
          }
          const auto frame_start = Clock::now();
          if (realtime)
          {
            const double target = sync_sim + std::chrono::duration<double>(Clock::now() - sync_wall).count();
            int n = 0;
            while (env.time() < target && n < 200 && !diverged)
            {
              stepOnce();
              ++n;
            }
            if (n >= 200)
            { // 跟不上实时：重新同步，避免越积越多
              sync_wall = Clock::now();
              sync_sim = env.time();
            }
          }
          else
          {
            while (std::chrono::duration<double>(Clock::now() - frame_start).count() < 0.015 && !diverged)
            {
              stepOnce();
            }
          }
        }
        const DualArmState &s = env.state();
        const char *status = diverged ? "\nDIVERGED" : (done ? "\nfinished (Esc to quit)" : (viewer.paused() ? "\npaused" : ""));
        if (s.screw.valid)
        {
          std::snprintf(title, sizeof(title), "time\nscene / ctrl\n|F weld| L / R\nobject drift\nscrew angle / feed%s",
                        *status ? "\nstatus" : "");
          std::snprintf(values, sizeof(values), "%.3f s\n%s / %s\n%.2f / %.2f N\n%.2f mm\n%.1f deg / %.3f mm%s", s.t,
                        cfg.scene.name.c_str(), controller->name(), s.arm(Arm::Left).weld_wrench.head<3>().norm(),
                        s.arm(Arm::Right).weld_wrench.head<3>().norm(), 1e3 * (s.object.pose.p - obj_p0).norm(),
                        s.screw.angle * 180.0 / M_PI, 1e3 * s.screw.feed, status);
        }
        else
        {
          std::snprintf(title, sizeof(title), "time\nscene / ctrl\n|F weld| L / R\nobject drift\nctrl mean%s",
                        *status ? "\nstatus" : "");
          std::snprintf(values, sizeof(values), "%.3f s\n%s / %s\n%.2f / %.2f N\n%.2f mm\n%.1f us%s", s.t,
                        cfg.scene.name.c_str(), controller->name(), s.arm(Arm::Left).weld_wrench.head<3>().norm(),
                        s.arm(Arm::Right).weld_wrench.head<3>().norm(), 1e3 * (s.object.pose.p - obj_p0).norm(),
                        st.steps ? st.ctrl_sum_us / st.steps : 0.0, status);
        }
        viewer.render(env.data(), title, values);
      }
#else
      throw std::runtime_error("built without viewer; use --headless");
#endif
    }
    logger.close();

    const double wall = std::chrono::duration<double>(Clock::now() - wall0).count();
    const DualArmState &fin = env.state();
    std::printf("[run_sim] done: t = %.3f s, %ld steps, wall %.2f s (x%.1f real time)\n", fin.t, st.steps, wall,
                wall > 0 ? fin.t / wall : 0.0);
    if (st.steps > 0)
    {
      std::printf("[run_sim] controller time: mean %.1f us, max %.1f us | sim step mean %.1f us\n",
                  st.ctrl_sum_us / st.steps, st.ctrl_max_us, st.step_sum_us / st.steps);
    }
    std::printf("[run_sim] object: max drift from initial position %.2f mm, final drift %.2f mm\n",
                1e3 * st.obj_dev_max, 1e3 * (fin.object.pose.p - obj_p0).norm());
    std::printf("[run_sim] max |weld force|: left %.2f N, right %.2f N | steps with torque saturation: %.0f\n",
                st.weld_force_max[0], st.weld_force_max[1], st.tau_sat_steps);
    if (fin.screw.valid)
    {
      std::printf("[run_sim] screw: angle %.2f deg, feed %.4f mm, resisting torque %.3f N·m\n",
                  fin.screw.angle * 180.0 / M_PI, 1e3 * fin.screw.feed, fin.screw.tauResist());
      monitor.update(fin);
      std::printf("[run_sim] axial preload: F/T %.2f N, weld %.2f N (target %.2f N)\n",
                  -monitor.value("work_ft_axial"), -monitor.value("work_weld_axial"),
                  cfg.asym_coop.axial_preload_force);
      if (asym_controller)
      {
        std::printf("[run_sim] assembly phase: %s | tightening torque ref/meas %.3f / %.3f N·m\n",
                    AsymmetricCoopController::phaseName(asym_controller->phase()),
                    asym_controller->tighteningTorqueReference(),
                    asym_controller->measuredTighteningTorque());
      }
      else if (qp_asym_controller)
      {
        std::printf(
            "[run_sim] assembly phase: %s | tightening torque ref/meas %.3f / %.3f N·m\n",
            AsymmetricCoopController::phaseName(qp_asym_controller->phase()),
            qp_asym_controller->tighteningTorqueReference(),
            qp_asym_controller->measuredTighteningTorque());
      }
    }
    auto print_qp_diagnostics = [](const auto &qp_controller)
    {
      std::printf(
          "[run_sim] torque QP: status %s | active constraints %d\n",
          qp_controller.qpStatus() == JointSafetyTorqueQp::Status::Solved
              ? "SOLVED"
              : "NOT_SOLVED",
          qp_controller.activeTorqueConstraints());
      std::printf(
          "[run_sim] torque QP: iterations %d | max constraint violation %.3e\n",
          qp_controller.qpIterations(),
          qp_controller.maxConstraintViolation());
      std::printf(
          "[run_sim] closed-chain acceleration residual: %.3e\n",
          qp_controller.closedChainAccelerationResidual());
      std::printf(
          "[run_sim] collision QP: active %d | min distance %.2f mm | max slack %.3e\n",
          qp_controller.activeCollisionConstraints(),
          1e3 * qp_controller.collisionMinDistance(),
          qp_controller.maxCollisionSlack());
    };
    if (qp_coop_controller)
    {
      print_qp_diagnostics(*qp_coop_controller);
      std::printf(
          "[run_sim] reference governor: phase %s | offset %.2f mm | virtual time %.3f s\n",
          QpCoopController::governorPhaseName(qp_coop_controller->governorPhase()),
          1e3 * qp_coop_controller->governorOffset(),
          qp_coop_controller->governorVirtualTime());
      std::printf(
          "[run_sim] reference governor summary: triggers %d | max offset %.2f mm | active %.3f s\n",
          st.governor_events,
          1e3 * st.governor_offset_max,
          st.governor_active_steps * cfg.timestep);
    }
    else if (qp_asym_controller)
      print_qp_diagnostics(*qp_asym_controller);
    if (st.d_min < 1e8)
      std::printf("[run_sim] min obstacle distance over logged steps: %.1f mm\n", 1e3 * st.d_min);
    if (!run_dir.empty())
      std::printf("[run_sim] log: %s\n", run_dir.c_str());
    if (diverged)
    {
      std::fprintf(stderr, "[run_sim] SIMULATION DIVERGED at t = %.4f s\n", fin.t);
      return 1;
    }
  }
  catch (const std::exception &e)
  {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
