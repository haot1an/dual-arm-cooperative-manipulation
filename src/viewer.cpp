#include "dual_arm/viewer.hpp"

#include <GLFW/glfw3.h>
#include <zlib.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace dual_arm {
namespace {

void putU32(std::vector<unsigned char>& out, std::uint32_t v) {
  out.push_back(static_cast<unsigned char>(v >> 24));
  out.push_back(static_cast<unsigned char>(v >> 16));
  out.push_back(static_cast<unsigned char>(v >> 8));
  out.push_back(static_cast<unsigned char>(v));
}

void putChunk(std::vector<unsigned char>& png, const char type[4], const unsigned char* data,
              std::size_t n) {
  putU32(png, static_cast<std::uint32_t>(n));
  const std::size_t start = png.size();
  png.insert(png.end(), type, type + 4);
  if (n) png.insert(png.end(), data, data + n);
  const uLong crc = crc32(0L, png.data() + start, static_cast<uInt>(n + 4));
  putU32(png, static_cast<std::uint32_t>(crc));
}

/// 写 8 位 RGB PNG（rows 自上而下），用 zlib 压缩
bool writePng(const std::string& path, int w, int h, const std::vector<unsigned char>& rows) {
  std::vector<unsigned char> raw;
  raw.reserve(static_cast<std::size_t>(h) * (3 * w + 1));
  for (int y = 0; y < h; ++y) {
    raw.push_back(0);  // filter: none
    const unsigned char* r = rows.data() + static_cast<std::size_t>(y) * 3 * w;
    raw.insert(raw.end(), r, r + 3 * static_cast<std::size_t>(w));
  }
  uLongf zlen = compressBound(static_cast<uLong>(raw.size()));
  std::vector<unsigned char> z(zlen);
  if (compress2(z.data(), &zlen, raw.data(), static_cast<uLong>(raw.size()), 9) != Z_OK) return false;

  std::vector<unsigned char> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
  std::vector<unsigned char> ihdr;
  putU32(ihdr, static_cast<std::uint32_t>(w));
  putU32(ihdr, static_cast<std::uint32_t>(h));
  ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});  // 8 bit, RGB, deflate, no filter, no interlace
  putChunk(png, "IHDR", ihdr.data(), ihdr.size());
  putChunk(png, "IDAT", z.data(), zlen);
  putChunk(png, "IEND", nullptr, 0);

  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  const bool ok = std::fwrite(png.data(), 1, png.size(), f) == png.size();
  std::fclose(f);
  return ok;
}

bool endsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

Viewer::Viewer(const mjModel* m, const Options& opt) : m_(m) {
  if (!glfwInit()) throw std::runtime_error("glfwInit failed (no display?)");
  glfwWindowHint(GLFW_VISIBLE, opt.visible ? GLFW_TRUE : GLFW_FALSE);
  glfwWindowHint(GLFW_SAMPLES, 4);
  window_ = glfwCreateWindow(opt.width, opt.height, opt.title.c_str(), nullptr, nullptr);
  if (!window_) {
    glfwTerminate();
    throw std::runtime_error("glfwCreateWindow failed");
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
  mjv_makeScene(m_, &scn_, 5000);
  mjr_makeContext(m_, &con_, mjFONTSCALE_150);

  mjv_defaultFreeCamera(m_, &cam_);
  if (!opt.camera.empty()) {
    const int cam_id = mj_name2id(m_, mjOBJ_CAMERA, opt.camera.c_str());
    if (cam_id >= 0) {
      cam_.type = mjCAMERA_FIXED;
      cam_.fixedcamid = cam_id;
    } else {
      std::fprintf(stderr, "[viewer] camera '%s' not found, using free camera\n", opt.camera.c_str());
    }
  }

  glfwSetWindowUserPointer(window_, this);
  glfwSetKeyCallback(window_, keyCallback);
  glfwSetMouseButtonCallback(window_, mouseButtonCallback);
  glfwSetCursorPosCallback(window_, cursorPosCallback);
  glfwSetScrollCallback(window_, scrollCallback);
}

Viewer::~Viewer() {
  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);
  if (window_) glfwDestroyWindow(window_);
  glfwTerminate();
}

bool Viewer::shouldClose() const { return glfwWindowShouldClose(window_); }

void Viewer::render(mjData* d, const char* overlay_title, const char* overlay_values) {
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
  mjr_render(viewport, &scn_, &con_);
  if (overlay_title && overlay_values) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, overlay_title, overlay_values, &con_);
  }
  mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
              "Space: pause   F: frames   C: contacts   V: next camera   T: transparent   Esc: quit",
              nullptr, &con_);
  glfwSwapBuffers(window_);
  glfwPollEvents();
}

void Viewer::renderOffscreen(mjData* d, std::vector<unsigned char>& rgb, int* width, int* height) {
  mjr_setBuffer(mjFB_OFFSCREEN, &con_);
  const mjrRect viewport = {0, 0, con_.offWidth, con_.offHeight};
  mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
  mjr_render(viewport, &scn_, &con_);
  rgb.resize(3 * static_cast<std::size_t>(viewport.width) * viewport.height);
  mjr_readPixels(rgb.data(), nullptr, viewport, &con_);
  mjr_setBuffer(mjFB_WINDOW, &con_);
  *width = viewport.width;
  *height = viewport.height;
}

bool Viewer::saveScreenshot(mjData* d, const std::string& path) {
  std::vector<unsigned char> rgb;
  int w = 0, h = 0;
  renderOffscreen(d, rgb, &w, &h);

  // OpenGL 的行顺序自下而上，图片文件自上而下
  std::vector<unsigned char> rows(rgb.size());
  const std::size_t stride = 3 * static_cast<std::size_t>(w);
  for (int y = 0; y < h; ++y) {
    std::copy_n(rgb.data() + static_cast<std::size_t>(h - 1 - y) * stride, stride,
                rows.data() + static_cast<std::size_t>(y) * stride);
  }
  if (endsWith(path, ".png") || endsWith(path, ".PNG")) return writePng(path, w, h, rows);

  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  std::fprintf(f, "P6\n%d %d\n255\n", w, h);
  const bool ok = std::fwrite(rows.data(), 1, rows.size(), f) == rows.size();
  std::fclose(f);
  return ok;
}

void Viewer::keyCallback(GLFWwindow* w, int key, int /*scancode*/, int act, int /*mods*/) {
  if (act != GLFW_PRESS) return;
  auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(w));
  switch (key) {
    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(w, GLFW_TRUE);
      break;
    case GLFW_KEY_SPACE:
      self->paused_ = !self->paused_;
      break;
    case GLFW_KEY_F:
      self->opt_.frame = (self->opt_.frame == mjFRAME_SITE) ? mjFRAME_NONE : mjFRAME_SITE;
      break;
    case GLFW_KEY_C:
      self->opt_.flags[mjVIS_CONTACTPOINT] = !self->opt_.flags[mjVIS_CONTACTPOINT];
      self->opt_.flags[mjVIS_CONTACTFORCE] = !self->opt_.flags[mjVIS_CONTACTFORCE];
      break;
    case GLFW_KEY_T:
      self->opt_.flags[mjVIS_TRANSPARENT] = !self->opt_.flags[mjVIS_TRANSPARENT];
      break;
    case GLFW_KEY_V:  // 自由相机 → 模型中的固定相机（按定义顺序）→ 自由相机
      if (self->cam_.type == mjCAMERA_FREE && self->m_->ncam > 0) {
        self->cam_.type = mjCAMERA_FIXED;
        self->cam_.fixedcamid = 0;
      } else if (self->cam_.type == mjCAMERA_FIXED && self->cam_.fixedcamid + 1 < self->m_->ncam) {
        self->cam_.fixedcamid += 1;
      } else {
        self->cam_.type = mjCAMERA_FREE;
      }
      break;
    default:
      break;
  }
}

void Viewer::mouseButtonCallback(GLFWwindow* w, int /*button*/, int /*act*/, int /*mods*/) {
  auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(w));
  self->button_left_ = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  self->button_middle_ = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  self->button_right_ = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(w, &self->last_x_, &self->last_y_);
}

void Viewer::cursorPosCallback(GLFWwindow* w, double x, double y) {
  auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(w));
  if (!self->button_left_ && !self->button_middle_ && !self->button_right_) return;
  const double dx = x - self->last_x_;
  const double dy = y - self->last_y_;
  self->last_x_ = x;
  self->last_y_ = y;
  int width = 1, height = 1;
  glfwGetWindowSize(w, &width, &height);
  const bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                     glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  int action;
  if (self->button_right_) {
    action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (self->button_left_) {
    action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }
  if (self->cam_.type == mjCAMERA_FIXED) self->cam_.type = mjCAMERA_FREE;
  mjv_moveCamera(self->m_, action, dx / height, dy / height, &self->cam_);
}

void Viewer::scrollCallback(GLFWwindow* w, double /*dx*/, double dy) {
  auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(w));
  if (self->cam_.type == mjCAMERA_FIXED) self->cam_.type = mjCAMERA_FREE;
  mjv_moveCamera(self->m_, mjMOUSE_ZOOM, 0, -0.05 * dy, &self->cam_);
}

VideoRecorder::VideoRecorder(const std::string& path, int width, int height, double fps)
    : frame_bytes_(3 * static_cast<std::size_t>(width) * height) {
  const std::string cmd = "ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -s " + std::to_string(width) + "x" +
                          std::to_string(height) + " -r " + std::to_string(fps) +
                          " -i - -vf vflip -c:v libx264 -preset fast -crf 20 -pix_fmt yuv420p '" + path + "'";
  pipe_ = popen(cmd.c_str(), "w");
  if (!pipe_) throw std::runtime_error("cannot start ffmpeg (is it installed?)");
}

VideoRecorder::~VideoRecorder() { close(); }

bool VideoRecorder::write(const std::vector<unsigned char>& rgb) {
  if (!pipe_ || rgb.size() != frame_bytes_) return false;
  const bool ok = std::fwrite(rgb.data(), 1, rgb.size(), pipe_) == rgb.size();
  if (ok) ++frames_;
  return ok;
}

bool VideoRecorder::close() {
  if (!pipe_) return true;
  const int rc = pclose(pipe_);
  pipe_ = nullptr;
  return rc == 0;
}

}  // namespace dual_arm
