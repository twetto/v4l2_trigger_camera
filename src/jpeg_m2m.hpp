#pragma once

#include <cerrno>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

//  Greyscale to JPEG on the VideoCore (bcm2835-codec "encode_image"), driven
//  directly through the V4L2 memory-to-memory API.
//
//  Measured on 640x400 at 59 Hz: about 14% of one core for capture, conversion
//  and encode together, against about 51% for libjpeg-turbo at quality 80.
//  GStreamer's v4l2jpegenc ignores the quality control, which is the other
//  reason to drive the device here: VIDIOC_S_CTRL on this file descriptor works.
//
//  The encoder takes no greyscale, only I420 and friends. Its chroma planes are
//  filled with neutral 128 once at start-up and never touched again, so each
//  frame costs one copy of the luma plane.

class JpegM2M
{
public:
  //  The node is found by name, since /dev/videoN renumbers.
  static std::string findDevice()
  {
    DIR * dir = ::opendir("/sys/class/video4linux");
    if (!dir) { return {}; }
    std::string found;
    while (auto * e = ::readdir(dir)) {
      std::ifstream f(std::string("/sys/class/video4linux/") + e->d_name + "/name");
      std::string name;
      if (std::getline(f, name) && name == "bcm2835-codec-encode_image") {
        found = std::string("/dev/") + e->d_name;
        break;
      }
    }
    ::closedir(dir);
    return found;
  }

  JpegM2M(const std::string & device, uint32_t width, uint32_t height, int quality)
  : width_(width), height_(height)
  {
    fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) { throw std::runtime_error("open " + device + ": " + std::strerror(errno)); }

    struct v4l2_format out {};
    out.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    out.fmt.pix_mp.width = width;
    out.fmt.pix_mp.height = height;
    out.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420;
    out.fmt.pix_mp.field = V4L2_FIELD_NONE;
    out.fmt.pix_mp.num_planes = 1;
    xioctl(VIDIOC_S_FMT, &out, "VIDIOC_S_FMT(output)");
    if (out.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_YUV420 || out.fmt.pix_mp.num_planes != 1) {
      throw std::runtime_error("JPEG encoder refused single-plane I420");
    }
    luma_stride_ = out.fmt.pix_mp.plane_fmt[0].bytesperline;

    struct v4l2_format cap {};
    cap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    cap.fmt.pix_mp.width = width;
    cap.fmt.pix_mp.height = height;
    cap.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_JPEG;
    cap.fmt.pix_mp.num_planes = 1;
    xioctl(VIDIOC_S_FMT, &cap, "VIDIOC_S_FMT(capture)");
    if (cap.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_JPEG) {
      throw std::runtime_error("JPEG encoder refused JPEG output");
    }

    setQuality(quality);

    out_ = allocate(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 1);
    cap_ = allocate(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 1);
    //  Neutral chroma, and the luma rows past `height` if the codec pads.
    std::memset(out_[0].start, 128, out_[0].length);

    queueCapture(0);
    enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    xioctl(VIDIOC_STREAMON, &t, "VIDIOC_STREAMON(output)");
    t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    xioctl(VIDIOC_STREAMON, &t, "VIDIOC_STREAMON(capture)");
  }

  ~JpegM2M()
  {
    if (fd_ >= 0) {
      enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
      ::ioctl(fd_, VIDIOC_STREAMOFF, &t);
      t = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      ::ioctl(fd_, VIDIOC_STREAMOFF, &t);
    }
    for (auto * v : {&out_, &cap_}) {
      for (auto & b : *v) {
        if (b.start) { ::munmap(b.start, b.length); }
      }
    }
    if (fd_ >= 0) { ::close(fd_); }
  }

  JpegM2M(const JpegM2M &) = delete;
  JpegM2M & operator=(const JpegM2M &) = delete;

  bool setQuality(int quality)
  {
    struct v4l2_control c {};
    c.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
    c.value = quality;
    return ::ioctl(fd_, VIDIOC_S_CTRL, &c) >= 0;
  }

  //  Synchronous: one frame in, one JPEG out. The encoder is idle between
  //  frames, so pipelining would buy nothing at this frame rate.
  void encode(const uint8_t * gray, uint32_t gray_stride, std::vector<uint8_t> & jpeg,
    int timeout_ms = 200)
  {
    auto * dst = static_cast<uint8_t *>(out_[0].start);
    for (uint32_t y = 0; y < height_; y++) {
      std::memcpy(dst + static_cast<size_t>(y) * luma_stride_,
        gray + static_cast<size_t>(y) * gray_stride, width_);
    }

    struct v4l2_plane plane {};
    plane.bytesused = static_cast<uint32_t>(out_[0].length);
    struct v4l2_buffer ob {};
    ob.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    ob.memory = V4L2_MEMORY_MMAP;
    ob.index = 0;
    ob.length = 1;
    ob.m.planes = &plane;
    xioctl(VIDIOC_QBUF, &ob, "VIDIOC_QBUF(output)");

    waitFor(POLLIN, timeout_ms, "JPEG encoder produced nothing");
    struct v4l2_plane cp {};
    struct v4l2_buffer cb {};
    cb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    cb.memory = V4L2_MEMORY_MMAP;
    cb.length = 1;
    cb.m.planes = &cp;
    xioctl(VIDIOC_DQBUF, &cb, "VIDIOC_DQBUF(capture)");
    const auto * src = static_cast<const uint8_t *>(cap_[cb.index].start) + cp.data_offset;
    jpeg.assign(src, src + (cp.bytesused - cp.data_offset));
    queueCapture(cb.index);

    //  Reclaim the input buffer before the next frame reuses it.
    waitFor(POLLOUT, timeout_ms, "JPEG encoder kept the input buffer");
    struct v4l2_plane rp {};
    struct v4l2_buffer rb {};
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    rb.memory = V4L2_MEMORY_MMAP;
    rb.length = 1;
    rb.m.planes = &rp;
    xioctl(VIDIOC_DQBUF, &rb, "VIDIOC_DQBUF(output)");
  }

private:
  struct MappedBuffer
  {
    void * start{nullptr};
    size_t length{0};
  };

  std::vector<MappedBuffer> allocate(uint32_t type, uint32_t count)
  {
    struct v4l2_requestbuffers req {};
    req.count = count;
    req.type = type;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(VIDIOC_REQBUFS, &req, "VIDIOC_REQBUFS");
    if (req.count < 1) { throw std::runtime_error("JPEG encoder allocated no buffers"); }
    std::vector<MappedBuffer> bufs(req.count);
    for (uint32_t i = 0; i < req.count; i++) {
      struct v4l2_plane plane {};
      struct v4l2_buffer buf {};
      buf.type = type;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      buf.length = 1;
      buf.m.planes = &plane;
      xioctl(VIDIOC_QUERYBUF, &buf, "VIDIOC_QUERYBUF");
      bufs[i].length = plane.length;
      bufs[i].start = ::mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
          plane.m.mem_offset);
      if (bufs[i].start == MAP_FAILED) {
        bufs[i].start = nullptr;
        throw std::runtime_error(std::string("mmap: ") + std::strerror(errno));
      }
    }
    return bufs;
  }

  void queueCapture(uint32_t index)
  {
    struct v4l2_plane plane {};
    struct v4l2_buffer b {};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = index;
    b.length = 1;
    b.m.planes = &plane;
    xioctl(VIDIOC_QBUF, &b, "VIDIOC_QBUF(capture)");
  }

  void waitFor(short events, int timeout_ms, const char * what)
  {
    struct pollfd pfd {};
    pfd.fd = fd_;
    pfd.events = events;
    int r;
    do {
      r = ::poll(&pfd, 1, timeout_ms);
    } while (r < 0 && errno == EINTR);
    if (r <= 0) { throw std::runtime_error(what); }
  }

  void xioctl(unsigned long request, void * arg, const char * what)
  {
    int r;
    do {
      r = ::ioctl(fd_, request, arg);
    } while (r < 0 && errno == EINTR);
    if (r < 0) { throw std::runtime_error(std::string(what) + ": " + std::strerror(errno)); }
  }

  int fd_{-1};
  uint32_t width_{0};
  uint32_t height_{0};
  uint32_t luma_stride_{0};
  std::vector<MappedBuffer> out_;
  std::vector<MappedBuffer> cap_;
};
