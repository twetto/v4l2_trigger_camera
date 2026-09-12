#pragma once

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

//  Thin V4L2 MJPEG capture wrapper. Deliberately free of OpenCV: the point of
//  this path is to hand the camera's own JPEG bytes straight to ROS without a
//  decode/re-encode round trip.
//
//  The setup order below matters on the Arducam OV9281. S_FMT, then S_PARM,
//  then controls, then REQBUFS, then STREAMON. Reordering these was observed to
//  leave triggered capture not working.

class V4L2Device
{
public:
  struct Frame
  {
    const uint8_t * data;
    size_t size;
    struct timeval timestamp;   // CLOCK_MONOTONIC
    uint32_t sequence;
    uint32_t index;
  };

  V4L2Device(const std::string & device, uint32_t width, uint32_t height, uint32_t fps)
  {
    fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("open " + device + ": " + std::strerror(errno));
    }

    struct v4l2_capability cap {};
    xioctl(VIDIOC_QUERYCAP, &cap, "VIDIOC_QUERYCAP");
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
      throw std::runtime_error(device + " is not a video capture device");
    }

    struct v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    xioctl(VIDIOC_S_FMT, &fmt, "VIDIOC_S_FMT");
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
      throw std::runtime_error("driver refused MJPEG");
    }
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;

    struct v4l2_streamparm parm {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    xioctl(VIDIOC_S_PARM, &parm, "VIDIOC_S_PARM");
  }

  ~V4L2Device()
  {
    if (streaming_) {
      try { stop(); } catch (...) {}
    }
    for (auto & b : buffers_) {
      if (b.start) { ::munmap(b.start, b.length); }
    }
    if (fd_ >= 0) { ::close(fd_); }
  }

  V4L2Device(const V4L2Device &) = delete;
  V4L2Device & operator=(const V4L2Device &) = delete;

  uint32_t width() const {return width_;}
  uint32_t height() const {return height_;}

  //  Returns false when the control does not exist, which varies by camera; the
  //  caller decides whether that is fatal.
  bool setControl(uint32_t id, int32_t value)
  {
    struct v4l2_control ctrl {};
    ctrl.id = id;
    ctrl.value = value;
    return ::ioctl(fd_, VIDIOC_S_CTRL, &ctrl) >= 0;
  }

  bool getControl(uint32_t id, int32_t & value) const
  {
    struct v4l2_control ctrl {};
    ctrl.id = id;
    if (::ioctl(fd_, VIDIOC_G_CTRL, &ctrl) < 0) { return false; }
    value = ctrl.value;
    return true;
  }

  void start(uint32_t buffer_count)
  {
    struct v4l2_requestbuffers req {};
    req.count = buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS");
    if (req.count < 1) {
      throw std::runtime_error("driver allocated no buffers");
    }

    buffers_.resize(req.count);
    for (uint32_t i = 0; i < req.count; i++) {
      struct v4l2_buffer buf {};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      xioctl(VIDIOC_QUERYBUF, &buf, "VIDIOC_QUERYBUF");

      buffers_[i].length = buf.length;
      buffers_[i].start = ::mmap(
        nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
        buffers_[i].start = nullptr;
        throw std::runtime_error(std::string("mmap: ") + std::strerror(errno));
      }
      queue(i);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
    streaming_ = true;
  }

  void stop()
  {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(VIDIOC_STREAMOFF, &type, "VIDIOC_STREAMOFF");
    streaming_ = false;
  }

  //  0 = timed out with no frame, 1 = frame in `frame`, -1 = interrupted.
  int wait(Frame & frame, int timeout_ms)
  {
    struct pollfd pfd {};
    pfd.fd = fd_;
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
    if (::ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
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
    xioctl(VIDIOC_QBUF, &buf, "VIDIOC_QBUF");
  }

private:
  struct MappedBuffer
  {
    void * start{nullptr};
    size_t length{0};
  };

  void xioctl(unsigned long request, void * arg, const char * what)
  {
    int r;
    do {
      r = ::ioctl(fd_, request, arg);
    } while (r < 0 && errno == EINTR);
    if (r < 0) {
      throw std::runtime_error(std::string(what) + ": " + std::strerror(errno));
    }
  }

  int fd_{-1};
  uint32_t width_{0};
  uint32_t height_{0};
  bool streaming_{false};
  std::vector<MappedBuffer> buffers_;
};
