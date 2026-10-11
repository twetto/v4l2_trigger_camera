#pragma once

#include <cerrno>
#include <cstring>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include "v4l2_device.hpp"

//  Greyscale capture from the Arducam UC-580 (OV9281 on one CSI-2 lane) through
//  the Pi's Unicam receiver, on linux-rpi with the uc580 overlay.
//
//  Unicam is a media-controller device: the sensor sub-device carries the
//  format and all the controls, the video node only the memory layout. Both are
//  found by name, because /dev/mediaN, /dev/videoN and /dev/v4l-subdevN all
//  renumber between boots.

struct CsiPaths
{
  std::string media;
  std::string video;
  std::string subdev;
  std::string sensor;     // entity name, e.g. "ov9281 10-0060"
};

namespace csi_detail
{
inline std::string devnodeFor(uint32_t major, uint32_t minor)
{
  //  The kernel names the node; /sys/dev/char/M:m/uevent says what it called it.
  std::ifstream f("/sys/dev/char/" + std::to_string(major) + ":" + std::to_string(minor) + "/uevent");
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("DEVNAME=", 0) == 0) { return "/dev/" + line.substr(8); }
  }
  return {};
}
}  // namespace csi_detail

//  A media device driven by "unicam" with an "ov9281" sensor entity. Absent on
//  any Pi whose config.txt serial filter did not apply the overlay.
inline std::optional<CsiPaths> findCsiCamera()
{
  DIR * dir = ::opendir("/dev");
  if (!dir) { return std::nullopt; }
  std::vector<std::string> medias;
  while (auto * e = ::readdir(dir)) {
    if (std::strncmp(e->d_name, "media", 5) == 0) { medias.push_back(std::string("/dev/") + e->d_name); }
  }
  ::closedir(dir);

  for (const auto & m : medias) {
    const int fd = ::open(m.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) { continue; }
    struct media_device_info info {};
    if (::ioctl(fd, MEDIA_IOC_DEVICE_INFO, &info) < 0 ||
      std::strcmp(info.driver, "unicam") != 0)
    {
      ::close(fd);
      continue;
    }
    CsiPaths p;
    p.media = m;
    struct media_entity_desc ent {};
    ent.id = MEDIA_ENT_ID_FLAG_NEXT;
    while (::ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &ent) == 0) {
      const std::string name = ent.name;
      if (name.rfind("ov9281", 0) == 0) {
        p.sensor = name;
        p.subdev = csi_detail::devnodeFor(ent.dev.major, ent.dev.minor);
      } else if (name == "unicam-image") {
        p.video = csi_detail::devnodeFor(ent.dev.major, ent.dev.minor);
      }
      ent.id |= MEDIA_ENT_ID_FLAG_NEXT;
    }
    ::close(fd);
    if (!p.sensor.empty() && !p.subdev.empty() && !p.video.empty()) { return p; }
  }
  return std::nullopt;
}

class CsiCamera
{
public:
  using Frame = V4L2Device::Frame;

  CsiCamera(const CsiPaths & paths, uint32_t width, uint32_t height)
  : paths_(paths)
  {
    if (!((width == 1280 && height == 800) || (width == 1280 && height == 720) ||
      (width == 640 && height == 400)))
    {
      throw std::runtime_error("UC-580 modes are 1280x800, 1280x720 and 640x400");
    }
    subdev_fd_ = openOrThrow(paths_.subdev);
    video_fd_ = openOrThrow(paths_.video);

    struct v4l2_subdev_format sf {};
    sf.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sf.pad = 0;
    sf.format.width = width;
    sf.format.height = height;
    sf.format.code = MEDIA_BUS_FMT_Y8_1X8;
    sf.format.field = V4L2_FIELD_NONE;
    xioctl(subdev_fd_, VIDIOC_SUBDEV_S_FMT, &sf, "VIDIOC_SUBDEV_S_FMT");
    if (sf.format.code != MEDIA_BUS_FMT_Y8_1X8 || sf.format.width != width ||
      sf.format.height != height)
    {
      throw std::runtime_error("sensor refused Y8 " + std::to_string(width) + "x" +
                               std::to_string(height));
    }

    struct v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    xioctl(video_fd_, VIDIOC_S_FMT, &fmt, "VIDIOC_S_FMT");
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_GREY) {
      throw std::runtime_error("unicam refused GREY");
    }
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    stride_ = fmt.fmt.pix.bytesperline;

    trigger_mode_ = readTriggerMode();
  }

  ~CsiCamera()
  {
    if (streaming_) {
      enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ::ioctl(video_fd_, VIDIOC_STREAMOFF, &type);
    }
    for (auto & b : buffers_) {
      if (b.start) { ::munmap(b.start, b.length); }
    }
    if (video_fd_ >= 0) { ::close(video_fd_); }
    if (subdev_fd_ >= 0) { ::close(subdev_fd_); }
  }

  CsiCamera(const CsiCamera &) = delete;
  CsiCamera & operator=(const CsiCamera &) = delete;

  uint32_t width() const {return width_;}
  uint32_t height() const {return height_;}
  uint32_t stride() const {return stride_;}
  const CsiPaths & paths() const {return paths_;}

  //  The overlay's trigger-mode property, read back from the sensor's own
  //  device-tree node. It is fixed at boot, so this only reports it.
  bool triggerMode() const {return trigger_mode_;}

  bool setControl(uint32_t id, int32_t value)
  {
    struct v4l2_control c {};
    c.id = id;
    c.value = value;
    return ::ioctl(subdev_fd_, VIDIOC_S_CTRL, &c) >= 0;
  }

  bool getControl(uint32_t id, int64_t & value) const
  {
    struct v4l2_ext_control c {};
    c.id = id;
    struct v4l2_ext_controls cs {};
    cs.which = V4L2_CTRL_WHICH_CUR_VAL;
    cs.count = 1;
    cs.controls = &c;
    if (::ioctl(subdev_fd_, VIDIOC_G_EXT_CTRLS, &cs) < 0) { return false; }
    //  PIXEL_RATE is the one 64-bit control read here.
    value = (id == V4L2_CID_PIXEL_RATE) ? c.value64 : c.value;
    return true;
  }

  bool controlMin(uint32_t id, int64_t & min) const
  {
    int64_t max = 0;
    return controlRange(id, min, max);
  }

  bool controlRange(uint32_t id, int64_t & min, int64_t & max) const
  {
    struct v4l2_query_ext_ctrl q {};
    q.id = id;
    if (::ioctl(subdev_fd_, VIDIOC_QUERY_EXT_CTRL, &q) < 0) { return false; }
    min = q.minimum;
    max = q.maximum;
    return true;
  }

  //  Exposure and gain in one call. The driver clusters them and writes both
  //  inside a sensor group hold, so they are meant to land on the same frame.
  bool setExposureGain(int32_t exposure_lines, int32_t gain)
  {
    struct v4l2_ext_control c[2] {};
    c[0].id = V4L2_CID_EXPOSURE;
    c[0].value = exposure_lines;
    c[1].id = V4L2_CID_ANALOGUE_GAIN;
    c[1].value = gain;
    struct v4l2_ext_controls cs {};
    cs.which = V4L2_CTRL_WHICH_CUR_VAL;
    cs.count = 2;
    cs.controls = c;
    return ::ioctl(subdev_fd_, VIDIOC_S_EXT_CTRLS, &cs) >= 0;
  }

  //  Line time and the exposure ceiling for the current frame length. The
  //  driver keeps reporting the old exposure maximum after vertical_blanking
  //  shrinks (1797 at 640x400 with blanking 22, where the frame is 422 lines),
  //  so the ceiling is computed here rather than queried.
  void timing(double & line_us, int32_t & max_exposure_lines) const
  {
    int64_t hblank = 0, vblank = 0, rate = 0;
    getControl(V4L2_CID_HBLANK, hblank);
    getControl(V4L2_CID_VBLANK, vblank);
    getControl(V4L2_CID_PIXEL_RATE, rate);
    line_us = rate > 0 ? (static_cast<double>(width_ + hblank) / static_cast<double>(rate)) * 1e6 : 0.0;
    max_exposure_lines = static_cast<int32_t>(height_ + vblank) - 25;
  }

  void start(uint32_t buffer_count)
  {
    struct v4l2_requestbuffers req {};
    req.count = buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(video_fd_, VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS");
    if (req.count < 1) { throw std::runtime_error("unicam allocated no buffers"); }
    buffers_.resize(req.count);
    for (uint32_t i = 0; i < req.count; i++) {
      struct v4l2_buffer buf {};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      xioctl(video_fd_, VIDIOC_QUERYBUF, &buf, "VIDIOC_QUERYBUF");
      buffers_[i].length = buf.length;
      buffers_[i].start = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
          video_fd_, buf.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
        buffers_[i].start = nullptr;
        throw std::runtime_error(std::string("mmap: ") + std::strerror(errno));
      }
      queue(i);
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(video_fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
    streaming_ = true;
  }

  //  0 = timed out, 1 = frame, -1 = interrupted. Same contract as V4L2Device.
  //  The timestamp is Unicam's frame-start interrupt, CLOCK_MONOTONIC.
  int wait(Frame & frame, int timeout_ms)
  {
    struct pollfd pfd {};
    pfd.fd = video_fd_;
    pfd.events = POLLIN;
    const int r = ::poll(&pfd, 1, timeout_ms);
    if (r < 0) {
      if (errno == EINTR) { return -1; }
      throw std::runtime_error(std::string("poll: ") + std::strerror(errno));
    }
    if (r == 0) { return 0; }
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (::ioctl(video_fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) { return 0; }
      throw std::runtime_error(std::string("VIDIOC_DQBUF: ") + std::strerror(errno));
    }
    frame.data = static_cast<const uint8_t *>(buffers_[buf.index].start);
    frame.size = buf.bytesused;
    frame.timestamp = buf.timestamp;
    frame.sequence = buf.sequence;
    frame.index = buf.index;
    return 1;
  }

  void queue(uint32_t index)
  {
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    xioctl(video_fd_, VIDIOC_QBUF, &buf, "VIDIOC_QBUF");
  }

private:
  struct MappedBuffer
  {
    void * start{nullptr};
    size_t length{0};
  };

  static int openOrThrow(const std::string & path)
  {
    const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) { throw std::runtime_error("open " + path + ": " + std::strerror(errno)); }
    return fd;
  }

  static void xioctl(int fd, unsigned long request, void * arg, const char * what)
  {
    int r;
    do {
      r = ::ioctl(fd, request, arg);
    } while (r < 0 && errno == EINTR);
    if (r < 0) { throw std::runtime_error(std::string(what) + ": " + std::strerror(errno)); }
  }

  bool readTriggerMode() const
  {
    struct stat st {};
    if (::fstat(subdev_fd_, &st) < 0) { return false; }
    const std::string base = "/sys/dev/char/" + std::to_string(major(st.st_rdev)) + ":" +
      std::to_string(minor(st.st_rdev)) + "/device/of_node/trigger-mode";
    std::ifstream f(base, std::ios::binary);
    unsigned char b[4] {};
    if (!f.read(reinterpret_cast<char *>(b), 4)) { return false; }
    return (b[0] | b[1] | b[2] | b[3]) != 0;    // big-endian u32, nonzero = on
  }

  CsiPaths paths_;
  int subdev_fd_{-1};
  int video_fd_{-1};
  uint32_t width_{0};
  uint32_t height_{0};
  uint32_t stride_{0};
  bool trigger_mode_{false};
  bool streaming_{false};
  std::vector<MappedBuffer> buffers_;
};
