#pragma once
/**
 * @file viewer.hpp
 * @brief 基于 GLFW + MuJoCo 自带 OpenGL 渲染的简易可视化窗口（单独编译为 dual_arm_viewer，
 *        核心库不依赖 GLFW）。仿真在主线程推进，Viewer 只负责画图与鼠标/键盘交互。
 *
 * 操作：
 *   鼠标左键拖动 旋转视角 | 右键拖动 平移 | 滚轮 / 中键 缩放
 *   Space 暂停/继续 | F 显示/隐藏 site 坐标系 | C 显示/隐藏接触力 | V 在自由相机与各固定相机间切换
 *   T 显示/隐藏透明 | Esc 退出
 */
#include <mujoco/mujoco.h>

#include <cstdio>
#include <string>
#include <vector>

struct GLFWwindow;

namespace dual_arm {

class Viewer {
 public:
  struct Options {
    int width = 1280;
    int height = 900;
    bool visible = true;           ///< false：隐藏窗口（只用于离屏截图）
    std::string title = "dual_arm";
    std::string camera;            ///< 固定相机名，如 cam_iso / cam_front（空 = 自由相机）
  };

  Viewer(const mjModel* m, const Options& opt);
  ~Viewer();
  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;

  bool shouldClose() const;
  bool paused() const { return paused_; }
  void setPaused(bool p) { paused_ = p; }

  /// 更新场景、渲染一帧（带左上/右上角文字）、交换缓冲区并处理事件
  void render(mjData* d, const char* overlay_title, const char* overlay_values);

  /// 离屏渲染当前状态并保存图片：扩展名 .png 存 PNG（zlib 压缩），否则存 PPM（P6）。
  /// 分辨率由场景 <visual><global offwidth/offheight> 决定。
  bool saveScreenshot(mjData* d, const std::string& path);

  /// 离屏渲染一帧到 rgb（自下而上的行顺序，3 字节/像素），返回 (宽, 高) = offscreen 缓冲区尺寸
  void renderOffscreen(mjData* d, std::vector<unsigned char>& rgb, int* width, int* height);

 private:
  static void keyCallback(GLFWwindow* w, int key, int scancode, int act, int mods);
  static void mouseButtonCallback(GLFWwindow* w, int button, int act, int mods);
  static void cursorPosCallback(GLFWwindow* w, double x, double y);
  static void scrollCallback(GLFWwindow* w, double dx, double dy);

  const mjModel* m_;
  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  bool paused_ = false;
  bool button_left_ = false, button_middle_ = false, button_right_ = false;
  double last_x_ = 0.0, last_y_ = 0.0;
};

/// 把离屏渲染的帧通过管道交给 ffmpeg 编码为 H.264 视频（需要系统里有 ffmpeg）
class VideoRecorder {
 public:
  VideoRecorder(const std::string& path, int width, int height, double fps);
  ~VideoRecorder();
  VideoRecorder(const VideoRecorder&) = delete;
  VideoRecorder& operator=(const VideoRecorder&) = delete;
  /// rgb：自下而上的行顺序（与 renderOffscreen 一致，由 ffmpeg 的 vflip 翻转）
  bool write(const std::vector<unsigned char>& rgb);
  bool close();
  int frames() const { return frames_; }

 private:
  std::FILE* pipe_ = nullptr;
  std::size_t frame_bytes_ = 0;
  int frames_ = 0;
};

}  // namespace dual_arm
