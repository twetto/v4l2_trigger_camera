#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <linux/videodev2.h>
#include <time.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rtimulib_ros2/msg/camera_trigger.hpp>
#include <v4l2_trigger_camera/msg/camera_exposure.hpp>

#include "auto_exposure.hpp"
#include "csi_camera.hpp"
#include "jpeg_m2m.hpp"
#include "v4l2_device.hpp"

#include <cstdio>
#include <csetjmp>
extern "C" {
#include <jpeglib.h>
}

namespace {

//  Decoding here rather than in a separate relay node is what makes the VIO
//  pipeline affordable. The camera can only deliver 58.8 Hz as MJPEG -- its
//  YUYV modes top out at 10 fps and it offers no GREY at all -- so the decode
//  has to happen somewhere. Doing it in this process means the downstream node
//  gets mono8 directly, and we drop a Python interpreter, a second DDS hop and
//  two copies of every frame.
//
//  scale_denom uses libjpeg's DCT scaling: the image is scaled during the
//  inverse DCT and the full-size result is never materialised, so decoding to
//  half size costs far less than decoding then resizing.
struct JpegErrorMgr {
  jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

void jpegErrorExit(j_common_ptr cinfo)
{
  auto * err = reinterpret_cast<JpegErrorMgr *>(cinfo->err);
  longjmp(err->setjmp_buffer, 1);
}

//  Returns false on a malformed frame rather than aborting: a single corrupt
//  JPEG should drop one frame, not take down the camera node.
bool decodeGray(
  const unsigned char * data, size_t size, unsigned scale_denom,
  std::vector<uint8_t> & out, unsigned & w, unsigned & h)
{
  jpeg_decompress_struct cinfo;
  JpegErrorMgr jerr;
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = jpegErrorExit;
  if (setjmp(jerr.setjmp_buffer)) {
    jpeg_destroy_decompress(&cinfo);
    return false;
  }
  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, data, size);
  if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
    jpeg_destroy_decompress(&cinfo);
    return false;
  }
  cinfo.out_color_space = JCS_GRAYSCALE;
  cinfo.scale_num = 1;
  cinfo.scale_denom = scale_denom;
  cinfo.dct_method = JDCT_IFAST;
  jpeg_start_decompress(&cinfo);
  w = cinfo.output_width;
  h = cinfo.output_height;
  out.resize(static_cast<size_t>(w) * h);
  while (cinfo.output_scanline < cinfo.output_height) {
    JSAMPROW row = out.data() + static_cast<size_t>(cinfo.output_scanline) * w;
    jpeg_read_scanlines(&cinfo, &row, 1);
  }
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return true;
}

}  // namespace

namespace
{
//  V4L2 buffer timestamps are CLOCK_MONOTONIC while ROS headers are on the ROS
//  clock, so every frame needs the offset between them. Sampling both clocks per
//  frame rather than once at startup keeps this correct across NTP slew, and
//  costs two clock reads.
rclcpp::Time monotonicToRos(const struct timeval & mono, const rclcpp::Time & ros_now)
{
  struct timespec now_mono;
  clock_gettime(CLOCK_MONOTONIC, &now_mono);

  const int64_t now_mono_ns =
    static_cast<int64_t>(now_mono.tv_sec) * 1000000000LL + now_mono.tv_nsec;
  const int64_t buf_mono_ns =
    static_cast<int64_t>(mono.tv_sec) * 1000000000LL + mono.tv_usec * 1000LL;

  return ros_now - rclcpp::Duration::from_nanoseconds(now_mono_ns - buf_mono_ns);
}

bool isUvcDevice(const std::string & path)
{
  const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) { return false; }
  struct v4l2_capability cap {};
  const bool uvc = ::ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
    std::strcmp(reinterpret_cast<const char *>(cap.driver), "uvcvideo") == 0;
  ::close(fd);
  return uvc;
}

//  A JPEG that carries no Huffman table cannot be decoded by a generic consumer.
//  Some UVC cameras omit it, which would make direct CompressedImage publication
//  invalid; check once rather than assume.
bool hasHuffmanTable(const uint8_t * d, size_t n)
{
  for (size_t i = 0; i + 1 < n; i++) {
    if (d[i] == 0xFF && d[i + 1] == 0xC4) { return true; }
  }
  return false;
}
}  // namespace

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : Node("v4l2_trigger_camera")
  {
    //  One root disk boots two Pis: the rig with the UVC camera, and one with the
    //  UC-580 on CSI. "auto" picks whichever is present; both present is refused
    //  rather than guessed, since each needs its own calibration downstream.
    const auto source = declare_parameter<std::string>("camera_source", "auto");
    const auto device = declare_parameter<std::string>("device", "/dev/video0");
    const auto width = declare_parameter<int>("width", 1280);
    const auto height = declare_parameter<int>("height", 800);
    const auto fps = declare_parameter<int>("fps", 100);
    const auto buffer_count = declare_parameter<int>("buffer_count", 4);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera");

    const auto trigger_mode = declare_parameter<bool>("trigger_mode", true);
    const auto exposure_profile =
      declare_parameter<std::string>("exposure_profile", "manual_indoor");
    const auto exposure = declare_parameter<int>("exposure", 10);
    const auto gain = declare_parameter<int>("gain", 0);

    use_trigger_stamp_ = declare_parameter<bool>("use_trigger_stamp", true);
    trigger_max_delay_ms_ = declare_parameter<double>("trigger_max_delay_ms", 50.0);
    no_frame_timeout_ = declare_parameter<double>("no_frame_timeout_sec", 10.0);
    const auto stats_every = declare_parameter<int>("stats_every_frames", 300);
    //  Publish decoded mono8 alongside the JPEG passthrough. The compressed
    //  topic is unchanged and still carries the camera's original bytes, so
    //  remote visualisation and bag recording are unaffected.
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    //  libjpeg only supports integer DCT scaling ratios; 2 gives 640x400 from
    //  this camera's 1280x800, which is what the VIO config expects.
    raw_scale_denom_ = declare_parameter<int>("raw_scale_denom", 2);
    if (raw_scale_denom_ != 1 && raw_scale_denom_ != 2 &&
      raw_scale_denom_ != 4 && raw_scale_denom_ != 8)
    {
      RCLCPP_WARN(
        get_logger(), "raw_scale_denom %d is not 1/2/4/8; falling back to 2",
        raw_scale_denom_);
      raw_scale_denom_ = 2;
    }
    stats_every_ = std::max<int64_t>(1, stats_every);

    const auto csi = (source == "uvc") ? std::nullopt : findCsiCamera();
    //  Ask the driver, not the path: /dev/video0 is Unicam's own node on the CSI Pi.
    const bool uvc_present = isUvcDevice(device);
    if (source == "csi" && !csi) {
      throw std::runtime_error("camera_source=csi but no unicam/ov9281 media device");
    }
    if (source == "auto" && csi && uvc_present) {
      throw std::runtime_error(
              "both a CSI camera (" + csi->sensor + ") and " + device +
              " are present; set camera_source to csi or uvc");
    }
    if (source != "auto" && source != "csi" && source != "uvc") {
      throw std::runtime_error("camera_source must be auto, csi or uvc");
    }
    if (csi) {
      RCLCPP_INFO(
        get_logger(), "camera_source=%s -> CSI: %s on %s (%s)", source.c_str(),
        csi->sensor.c_str(), csi->video.c_str(), csi->media.c_str());
      setupCsi(*csi);
      return;
    }
    RCLCPP_INFO(get_logger(), "camera_source=%s -> UVC: %s", source.c_str(), device.c_str());

    camera_ = std::make_unique<V4L2Device>(
      device, static_cast<uint32_t>(width), static_cast<uint32_t>(height),
      static_cast<uint32_t>(fps));

    applyExposure(exposure_profile, exposure, gain);

    //  On this camera exposure_dynamic_framerate is what puts the sensor into
    //  externally triggered capture; it is not a frame-rate hint.
    if (!camera_->setControl(V4L2_CID_EXPOSURE_AUTO_PRIORITY, trigger_mode ? 1 : 0)) {
      RCLCPP_WARN(get_logger(), "could not set exposure_dynamic_framerate (trigger mode)");
    }
    if (exposure_profile == "closed_loop") { setupUvcAutoExposure(exposure, gain); }

    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "image/compressed", rclcpp::SensorDataQoS());
    if (publish_raw_) {
      //  Sensor-data QoS: the consumer is a VIO node that always wants the
      //  newest frame, and a late frame is worth less than a dropped one.
      raw_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS());
    }
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS());

    if (use_trigger_stamp_) {
      trigger_sub_ = create_subscription<rtimulib_ros2::msg::CameraTrigger>(
        "trigger", rclcpp::SensorDataQoS(),
        [this](rtimulib_ros2::msg::CameraTrigger::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(trigger_mutex_);
          noteTriggerPeriod(rclcpp::Time(msg->header.stamp));
          triggers_.push_back(*msg);
          if (triggers_.size() > 400) { triggers_.pop_front(); }
        });
    }

    //  Start the drought clock at stream start, not at first frame: the failure
    //  seen in practice was a node that opened the device cleanly and then never
    //  received anything at all.
    last_frame_ = std::chrono::steady_clock::now();
    camera_->start(static_cast<uint32_t>(std::max<int64_t>(1, buffer_count)));
    RCLCPP_INFO(
      get_logger(), "Capturing %ux%u MJPEG from %s, trigger_mode=%s, stamp=%s",
      camera_->width(), camera_->height(), device.c_str(), trigger_mode ? "on" : "off",
      !use_trigger_stamp_ ? "v4l2" : uvc_ae_ && mid_exposure_stamp_ ? "trigger+mid-exposure" : "trigger");

    worker_ = std::thread([this]() {captureLoop();});
  }

  ~CameraNode() override
  {
    running_ = false;
    if (worker_.joinable()) { worker_.join(); }
    {
      std::lock_guard<std::mutex> lock(uvc_write_mutex_);
    }
    uvc_write_cv_.notify_all();
    if (uvc_writer_.joinable()) { uvc_writer_.join(); }
  }

private:
  //  ---- CSI (UC-580) -------------------------------------------------------
  //
  //  Timing, measured on this sensor through Unicam (kernel/uc580-camera-rpi-dkms
  //  README in duckiebot-installation-guide):
  //  - the buffer timestamp is Unicam's frame-start interrupt;
  //  - frame start follows the trigger by 0.883 ms + 15.94 us x exposure lines,
  //    std ~1.4 us; the split of the 0.883 ms before/after exposure is unknown;
  //  - an exposure or gain write lands on the very next triggered frame.
  //  So the trigger-to-frame delay identifies the exposure each frame really
  //  used, and the published stamp can be put at mid-exposure.

  void setupCsi(const CsiPaths & paths)
  {
    const auto w = declare_parameter<int>("csi_width", 640);
    const auto h = declare_parameter<int>("csi_height", 400);
    //  -1: the mode's minimum. In trigger mode the pulses set the frame rate,
    //  and a shorter frame lowers the shortest pulse period the sensor can follow.
    const auto vblank = declare_parameter<int>("csi_vertical_blanking", -1);
    const auto quality = declare_parameter<int>("jpeg_quality", 80);
    auto_exposure_ = declare_parameter<bool>("auto_exposure", true);
    manual_.exposure_lines = declare_parameter<int>("csi_exposure", 300);
    manual_.gain = declare_parameter<int>("csi_gain", 32);
    declareAeParameters(16.0, 4);
    max_skip_rate_ = declare_parameter<double>("ae_max_skip_rate", 0.005);
    ae_.gain_max = declare_parameter<int>("ae_gain_max", 255);
    //  Low light: past the full-rate ceiling and this gain, exposure may grow up
    //  to ae_long_exposure_max_ms, with the sensor skipping pulses (frame rate
    //  drops, frames stay whole and matched). The cap is a motion-blur limit.
    //  0 turns it off.
    const double long_exposure_ms = declare_parameter<double>("ae_long_exposure_max_ms", 33.0);
    ae_.long_exposure_gain = declare_parameter<int>("ae_long_exposure_gain", 64);
    csi_latency_offset_ms_ = declare_parameter<double>("csi_latency_offset_ms", 0.883);
    csi_line_us_ = declare_parameter<double>("csi_exposure_line_us", 15.94);
    //  Trigger edge to exposure start. Unknown within [0, 0.883] ms; a constant
    //  here is absorbed by the downstream camera-IMU time-offset calibration.
    mid_exposure_offset_us_ = declare_parameter<double>("mid_exposure_offset_us", 0.0);

    csi_ = std::make_unique<CsiCamera>(paths, static_cast<uint32_t>(w), static_cast<uint32_t>(h));
    csi_->setControl(V4L2_CID_HFLIP, 0);    // the driver defaults both on: upside
    csi_->setControl(V4L2_CID_VFLIP, 0);    // down as the UC-580 is mounted
    int64_t vmin = 0, vmax = 0;
    csi_->controlRange(V4L2_CID_VBLANK, vmin, vmax);
    vblank_base_ = vblank < 0 ? static_cast<int32_t>(vmin) : static_cast<int32_t>(vblank);
    vblank_max_ = static_cast<int32_t>(vmax);
    vblank_cur_ = vblank_base_;
    csi_->setControl(V4L2_CID_VBLANK, vblank_base_);
    double hts_line_us = 0.0;
    csi_->timing(hts_line_us, exposure_ceiling_);
    exposure_frame_limit_ = exposure_ceiling_;
    ae_.exposure_max = exposure_ceiling_;
    {
      //  Also bounded by the frame length the driver allows, and by the
      //  trigger matcher: a frame arrives 0.883 ms + exposure after its pulse,
      //  and pulses older than trigger_max_delay_ms are not considered.
      const double usable_ms = std::min(
        long_exposure_ms, trigger_max_delay_ms_ - csi_latency_offset_ms_ - 2.0);
      const int32_t lines = static_cast<int32_t>(std::max(0.0, usable_ms) * 1000.0 / csi_line_us_);
      ae_.long_exposure_max = std::min<int32_t>(
        lines, static_cast<int32_t>(csi_->height()) + vblank_max_ - 25);
      if (long_exposure_ms > usable_ms) {
        RCLCPP_WARN(
          get_logger(), "ae_long_exposure_max_ms %.1f limited to %.1f by trigger_max_delay_ms %.1f",
          long_exposure_ms, usable_ms, trigger_max_delay_ms_);
      }
    }
    manual_.exposure_lines = std::clamp(manual_.exposure_lines, 1, exposure_ceiling_);
    current_ = manual_;
    writeExposure(current_);

    const std::string enc_dev = JpegM2M::findDevice();
    if (enc_dev.empty()) { throw std::runtime_error("no bcm2835-codec JPEG encoder"); }
    jpeg_ = std::make_unique<JpegM2M>(enc_dev, csi_->width(), csi_->height(), quality);

    image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "image/compressed", rclcpp::SensorDataQoS());
    if (publish_raw_) {
      raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS());
    }
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS());
    exposure_pub_ = create_publisher<v4l2_trigger_camera::msg::CameraExposure>(
      "exposure", rclcpp::SensorDataQoS());
    if (use_trigger_stamp_) {
      trigger_sub_ = create_subscription<rtimulib_ros2::msg::CameraTrigger>(
        "trigger", rclcpp::SensorDataQoS(),
        [this](rtimulib_ros2::msg::CameraTrigger::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(trigger_mutex_);
          noteTriggerPeriod(rclcpp::Time(msg->header.stamp));
          triggers_.push_back(*msg);
          if (triggers_.size() > 400) { triggers_.pop_front(); }
        });
    }

    last_frame_ = std::chrono::steady_clock::now();
    csi_->start(4);
    RCLCPP_INFO(
      get_logger(),
      "CSI %ux%u GREY, trigger_mode=%s (overlay), line %.3f us (timing model %.3f), "
      "exposure ceiling %d lines, JPEG quality %d on %s, auto_exposure=%s, stamp=%s",
      csi_->width(), csi_->height(), csi_->triggerMode() ? "on" : "off", hts_line_us,
      csi_line_us_, exposure_ceiling_, static_cast<int>(quality), enc_dev.c_str(),
      auto_exposure_ ? ae_mode_.c_str() : "off", use_trigger_stamp_ ? "trigger+mid-exposure" : "v4l2");
    worker_ = std::thread([this]() {csiLoop();});
  }

  //  Auto exposure parameters common to both sources. Only the defaults that
  //  are properties of the camera differ.
  void declareAeParameters(double black_level, int64_t meter_step)
  {
    ae_.target_level = declare_parameter<double>("ae_target_level", 100.0);
    ae_.black_level = declare_parameter<double>("ae_black_level", black_level);
    ae_.deadband = declare_parameter<double>("ae_deadband", 0.06);
    ae_.correction = declare_parameter<double>("ae_correction", 0.5);
    ae_.max_step_ratio = declare_parameter<double>("ae_max_step_ratio", 1.12);
    ae_.saturation_limit = declare_parameter<double>("ae_saturation_limit", 0.05);
    ae_.saturation_resume = declare_parameter<double>("ae_saturation_resume", 0.025);
    ae_.saturation_floor_level = declare_parameter<double>("ae_saturation_floor_level", 60.0);
    ae_meter_step_ = std::max<int64_t>(1, declare_parameter<int>("ae_meter_step", meter_step));
    //  mean: hold mean brightness at ae_target_level. gradient: maximise the
    //  soft-percentile gradient metric of Zhang et al. (ICRA 2017), adapted to
    //  this linear sensor; see auto_exposure.hpp.
    ae_mode_ = declare_parameter<std::string>("ae_mode", "mean");
    if (ae_mode_ != "mean" && ae_mode_ != "gradient") {
      throw std::runtime_error("ae_mode must be mean or gradient");
    }
    grad_.percentile = declare_parameter<double>("ae_gradient_percentile", 0.75);
    grad_.k = declare_parameter<int>("ae_gradient_k", 5);
    grad_.step = static_cast<uint32_t>(std::max<int64_t>(1, declare_parameter<int>("ae_gradient_step", 2)));
    grad_.gain_floor_level = declare_parameter<double>("ae_gain_floor_level", 60.0);
  }

  void writeExposure(const ExposureSetting & s)
  {
    //  The frame must be at least exposure + 25 lines long, and no longer than
    //  that: a longer frame raises the shortest pulse period the sensor can
    //  follow. The driver checks exposure against the current frame length,
    //  so a longer frame is set before the exposure, a shorter one after.
    const int32_t vblank = std::clamp(
      s.exposure_lines + 25 - static_cast<int32_t>(csi_->height()), vblank_base_, vblank_max_);
    if (vblank > vblank_cur_ && csi_->setControl(V4L2_CID_VBLANK, vblank)) { vblank_cur_ = vblank; }
    if (!csi_->setExposureGain(s.exposure_lines, s.gain)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "exposure/gain write failed");
      return;
    }
    if (vblank < vblank_cur_ && csi_->setControl(V4L2_CID_VBLANK, vblank)) { vblank_cur_ = vblank; }
    //  Remember when each write completed: a frame used the last write that
    //  finished before its trigger pulse.
    writes_.push_back({now(), s});
    if (writes_.size() > 32) { writes_.pop_front(); }
  }

  //  The setting a frame triggered at `trigger` used. Timing gives its exposure
  //  directly; gain is taken from the write that matches that exposure, or else
  //  the last one completed before the pulse.
  ExposureSetting settingFor(const rclcpp::Time & trigger, double measured_lines, bool & measured)
  {
    ExposureSetting s = current_;
    for (const auto & w : writes_) {
      if (w.done < trigger) { s = w.setting; }
    }
    measured = measured_lines > 0.0;
    if (!measured) { return s; }
    const int32_t lines = static_cast<int32_t>(std::lround(measured_lines));
    if (std::abs(lines - s.exposure_lines) > 2) {
      //  The write raced the pulse. Prefer the write whose exposure the timing
      //  shows; failing that, report the measured exposure with the gain guess.
      for (auto it = writes_.rbegin(); it != writes_.rend(); ++it) {
        if (std::abs(lines - it->setting.exposure_lines) <= 2) { return it->setting; }
      }
      s.exposure_lines = lines;
    }
    return s;
  }

  void csiLoop()
  {
    std::vector<uint8_t> jpeg;
    while (rclcpp::ok() && running_) {
      CsiCamera::Frame frame;
      int r;
      try {
        r = csi_->wait(frame, 500);
      } catch (const std::exception & err) {
        RCLCPP_ERROR(get_logger(), "capture failed, shutting down to respawn: %s", err.what());
        running_ = false;
        rclcpp::shutdown();
        return;
      }
      if (r <= 0) {
        if (r == 0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "no frame for 500 ms; is the trigger running?");
        }
        continue;
      }
      last_frame_ = std::chrono::steady_clock::now();
      const auto v4l2_time = monotonicToRos(frame.timestamp, now());

      if (have_last_seq_ && frame.sequence != last_seq_ + 1) {
        dropped_ += (frame.sequence - last_seq_ - 1);
      }
      last_seq_ = frame.sequence;
      have_last_seq_ = true;

      //  Copy out of the capture buffer once and hand it straight back. The
      //  Unicam buffer reads as uncached memory: the gradient metric took 35 ms
      //  a frame on it against 1.8 ms on a copy, and the memcpy is 0.4 ms. Every
      //  later step reads `pixels`.
      pixels_.resize(static_cast<size_t>(csi_->stride()) * csi_->height());
      std::memcpy(pixels_.data(), frame.data, pixels_.size());
      csi_->queue(frame.index);
      const uint8_t * pixels = pixels_.data();

      //  Meter and set the next frame's exposure first: the next pulse can be
      //  as little as ~2 ms away, and a write that misses it lands one frame late.
      double mean = 0.0, saturated = 0.0, metric = 0.0;
      meterGray(
        pixels, csi_->width(), csi_->height(), csi_->stride(),
        static_cast<uint32_t>(ae_meter_step_), mean, saturated);
      const ExposureSetting written_before = current_;
      if (auto_exposure_) {
        ae_.exposure_max = exposure_ceiling_;
        const ExposureSetting next = ae_mode_ == "gradient" ?
          gradientExposureStep(
          ae_, grad_, gradient_metric_, current_, pixels, csi_->width(), csi_->height(),
          csi_->stride(), mean, saturated, metric) :
          autoExposureStep(ae_, current_, mean, saturated);
        if (!(next == current_)) {
          current_ = next;
          writeExposure(current_);
        }
      }

      //  Match on the delay this frame's exposure predicts, then read the
      //  exposure it actually used back from the delay.
      rclcpp::Time stamp = v4l2_time;
      rtimulib_ros2::msg::CameraTrigger trig;
      double delay_ms = 0.0, measured_lines = -1.0;
      ExposureSetting used = written_before;
      bool measured = false;
      const double predicted_ms =
        csi_latency_offset_ms_ + written_before.exposure_lines * csi_line_us_ * 1e-3;
      const bool matched = use_trigger_stamp_ && matchTrigger(v4l2_time, trig, delay_ms, predicted_ms);
      if (matched) {
        measured_lines = (delay_ms - csi_latency_offset_ms_) / (csi_line_us_ * 1e-3);
        used = settingFor(rclcpp::Time(trig.header.stamp), measured_lines, measured);
        const double exposure_us = used.exposure_lines * csi_line_us_;
        stamp = rclcpp::Time(trig.header.stamp) + rclcpp::Duration::from_nanoseconds(
          static_cast<int64_t>((mid_exposure_offset_us_ + 0.5 * exposure_us) * 1e3));
        window_delays_.push_back(delay_ms);
        matched_++;
        noteSkips(trig.trigger_count, used);
      } else {
        //  No trigger to anchor to (free-running, or unmatched): frame start is
        //  shortly after exposure ends, so back off by half the exposure.
        stamp = v4l2_time - rclcpp::Duration::from_nanoseconds(
          static_cast<int64_t>(0.5 * used.exposure_lines * csi_line_us_ * 1e3));
        if (use_trigger_stamp_) { unmatched_++; }
      }

      try {
        jpeg_->encode(pixels, csi_->stride(), jpeg);
      } catch (const std::exception & err) {
        RCLCPP_ERROR(get_logger(), "JPEG encode failed, shutting down to respawn: %s", err.what());
        running_ = false;
        rclcpp::shutdown();
        return;
      }
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id_;
      msg->format = "jpeg";
      msg->data = jpeg;
      image_pub_->publish(std::move(msg));

      if (publish_raw_) {
        auto raw = std::make_unique<sensor_msgs::msg::Image>();
        raw->header.stamp = stamp;
        raw->header.frame_id = frame_id_;
        raw->height = csi_->height();
        raw->width = csi_->width();
        raw->encoding = "mono8";
        raw->is_bigendian = 0;
        raw->step = csi_->width();
        raw->data.resize(static_cast<size_t>(raw->width) * raw->height);
        for (uint32_t y = 0; y < raw->height; y++) {
          std::memcpy(
            raw->data.data() + static_cast<size_t>(y) * raw->width,
            pixels + static_cast<size_t>(y) * csi_->stride(), raw->width);
        }
        raw_pub_->publish(std::move(raw));
      }

      v4l2_trigger_camera::msg::CameraExposure ex;
      ex.header.stamp = stamp;
      ex.header.frame_id = frame_id_;
      ex.exposure_lines = static_cast<uint32_t>(used.exposure_lines);
      ex.exposure_us = static_cast<float>(used.exposure_lines * csi_line_us_);
      ex.analogue_gain = static_cast<uint32_t>(used.gain);
      ex.exposure_measured = measured;
      ex.auto_exposure = auto_exposure_;
      ex.mean_level = static_cast<float>(mean);
      ex.saturated_fraction = static_cast<float>(saturated);
      ex.gradient_metric = static_cast<float>(metric);
      exposure_pub_->publish(ex);

      sensor_msgs::msg::CameraInfo info;
      info.header.stamp = stamp;
      info.header.frame_id = frame_id_;
      info.width = csi_->width();
      info.height = csi_->height();
      info_pub_->publish(info);

      window_mean_ += mean;
      window_n_++;
      if (++frames_ % stats_every_ == 0) {
        reportStats();
        RCLCPP_INFO(
          get_logger(),
          "  exposure %d lines (%.2f ms) gain %d | mean %.1f sat %.2f%% | ceiling %d | "
          "skipped pulses %lu | jpeg %.1f KiB",
          current_.exposure_lines, current_.exposure_lines * csi_line_us_ * 1e-3, current_.gain,
          window_n_ ? window_mean_ / window_n_ : 0.0, saturated * 100.0, exposure_ceiling_,
          static_cast<unsigned long>(skipped_pulses_), jpeg.size() / 1024.0);
        window_mean_ = 0.0;
        window_n_ = 0;
      }
    }
  }

  //  A pulse with no frame. The sensor needs roughly frame time + exposure +
  //  2-3 ms between pulses, and the rig's pulses follow IMU sample arrival: on
  //  the rig every skipped pulse came 11.5-13.1 ms after the last captured one,
  //  against a 16.9 ms period. Catching all of them would cap exposure near 160
  //  lines (2.5 ms), and an instant ratchet chased each rarer gap lower (397 ->
  //  222 lines in 90 s). So a small skip rate is accepted -- a skipped pulse
  //  only means no frame, the others keep their timing -- and the ceiling moves
  //  per window of kSkipWindow pulses: down when skips exceed ae_max_skip_rate
  //  with exposure near the ceiling, slowly back up after a window with none.
  static constexpr uint64_t kSkipWindow = 600;

  void noteSkips(uint64_t trigger_count, const ExposureSetting & used)
  {
    //  Past the full-rate ceiling (low light) skips are the intended trade, not
    //  evidence about the ceiling: they are counted in the total only.
    const bool long_exposure = used.exposure_lines > exposure_ceiling_;
    if (long_exposure) {
      if (have_last_trigger_count_ && trigger_count > last_trigger_count_ + 1) {
        skipped_pulses_ += trigger_count - last_trigger_count_ - 1;
      }
      last_trigger_count_ = trigger_count;
      have_last_trigger_count_ = true;
      return;
    }
    if (have_last_trigger_count_ && trigger_count > last_trigger_count_) {
      window_pulses_ += trigger_count - last_trigger_count_;
    }
    if (have_last_trigger_count_ && trigger_count > last_trigger_count_ + 1) {
      skipped_pulses_ += trigger_count - last_trigger_count_ - 1;
      window_skips_ += trigger_count - last_trigger_count_ - 1;
      //  How soon after the last captured pulse did the skipped one come?
      double gap_ms = -1.0;
      {
        std::lock_guard<std::mutex> lock(trigger_mutex_);
        rclcpp::Time captured, skipped;
        bool have_c = false, have_s = false;
        for (const auto & t : triggers_) {
          if (t.trigger_count == last_trigger_count_) { captured = t.header.stamp; have_c = true; }
          if (t.trigger_count == last_trigger_count_ + 1) { skipped = t.header.stamp; have_s = true; }
        }
        if (have_c && have_s) { gap_ms = (skipped - captured).seconds() * 1e3; }
      }
      RCLCPP_INFO(
        get_logger(), "skipped pulse %lu came %.3f ms after the last captured one "
        "(period %.2f ms, exposure %d lines)",
        static_cast<unsigned long>(last_trigger_count_ + 1), gap_ms, trigger_period_ms_,
        used.exposure_lines);
      window_near_ceiling_ |= used.exposure_lines >= exposure_ceiling_ * 9 / 10;
    }
    last_trigger_count_ = trigger_count;
    have_last_trigger_count_ = true;

    if (window_pulses_ < kSkipWindow) { return; }
    const double rate = static_cast<double>(window_skips_) / static_cast<double>(window_pulses_);
    if (auto_exposure_ && rate > max_skip_rate_ && window_near_ceiling_) {
      const int32_t lowered = std::max(1, exposure_ceiling_ * 95 / 100);
      RCLCPP_WARN(
        get_logger(), "%lu of %lu pulses skipped (%.2f%% > %.2f%%) near the ceiling; "
        "lowering it %d -> %d lines", static_cast<unsigned long>(window_skips_),
        static_cast<unsigned long>(window_pulses_), rate * 100.0, max_skip_rate_ * 100.0,
        exposure_ceiling_, lowered);
      exposure_ceiling_ = lowered;
    } else if (window_skips_ == 0 && exposure_ceiling_ < exposure_frame_limit_) {
      exposure_ceiling_ = std::min(
        exposure_frame_limit_, exposure_ceiling_ + std::max(1, exposure_ceiling_ / 50));
    }
    window_pulses_ = 0;
    window_skips_ = 0;
    window_near_ceiling_ = false;
  }

  //  ---- UVC closed-loop exposure ---------------------------------------------
  //
  //  The CSI controller (auto_exposure.hpp) driven through UVC controls. The
  //  sensor is the same OV9281; what differs is the camera's bridge firmware,
  //  measured on the rig 2026-10-15 (320x200 decode, one indoor scene):
  //  - exposure_time_absolute is in 100 us units. Mean = 32.0 + 0.771 x exposure
  //    for 1..55; a soft knee to ~83, flat above (the control goes to 5000 but
  //    does nothing there). The ceiling sits at the knee so the loop's linear
  //    model holds. The frame rate halves between 47 (17.4 ms frame interval)
  //    and 52 (33.8 ms): uvc_ae_exposure_max 47 keeps the full rate at the cost
  //    of 15% less light.
  //  - the ~32 intercept is a black pedestal, not hidden exposure: spatial spread
  //    is 8 levels at exposure 1 against 88 at 50.
  //  - gain 0..100 is linear, 1 + 0.0189 per unit (2.89x at 100, fit at
  //    exposure 20), and multiplies the pedestal too (black_scales_with_gain).
  //  - a write issued right after a frame arrives lands all at once, on the
  //    first frame triggered more than 34 and about 51 ms after that frame's
  //    trigger: 3 frames later at 58.8 Hz (73 of 80 gain toggles), 2 at 29.7 Hz
  //    (39 of 40; and 251 of 254 gain steps in the dark walk bag
  //    vio_live_20261015-144842). So it is tracked in trigger time, not frames.
  //    The loop is tuned for 1 frame of delay; at 3 its 0.5 correction would
  //    oscillate (simulated limit cycle, +/-33%), and gradient mode, which steps
  //    a fixed 12%, would overshoot by the frames in flight. So after a write
  //    nothing is metered until it has landed, and the CSI steps run unchanged.
  //
  //  The controller works in the CSI's units: gain 16 = 1x, linear. The
  //  conversion to the UVC control is here, at the write.

  void setupUvcAutoExposure(int exposure, int gain)
  {
    declareAeParameters(32.0, 2);
    ae_.black_scales_with_gain = true;
    //  Clips at 244 at gain 0; 240 leaves room for JPEG noise around it. Held
    //  at every gain: if gain lifts the clip to 255 (not measured), this only
    //  counts the top 6% of the range as clipped and darkens a little early.
    ae_.white_level = declare_parameter<int>("ae_white_level", 240);
    ae_.exposure_min = 1;
    ae_.exposure_max = declare_parameter<int>("uvc_ae_exposure_max", 55);
    uvc_gain_slope_ = declare_parameter<double>("uvc_gain_slope", 0.0189);
    uvc_gain_max_ = declare_parameter<int>("uvc_ae_gain_max", 100);
    ae_.gain_min = 16;
    ae_.gain_max = static_cast<int32_t>(std::floor(16.0 * (1.0 + uvc_gain_slope_ * uvc_gain_max_)));
    ae_.long_exposure_max = 0;    // the bridge drops the frame rate by itself past the knee
    //  Milliseconds after the metered frame's trigger. Latency: midway in the
    //  measured (34, 51] window, which decides which frame is reported as the
    //  first with the new setting. Settle: past its end with margin, so the
    //  next frame metered has surely got it: 4 frames at 58.8 Hz, 2 at 29.7 Hz.
    uvc_write_latency_ms_ = declare_parameter<double>("uvc_ae_write_latency_ms", 42.0);
    uvc_settle_ms_ = declare_parameter<double>("uvc_ae_settle_ms", 60.0);
    //  Stamp at mid-exposure (trigger + exposure / 2) rather than at the
    //  trigger edge. With exposure under control it moves by up to 2.75 ms;
    //  the edge-to-exposure-start delay that remains is a constant.
    mid_exposure_stamp_ = declare_parameter<bool>("mid_exposure_stamp", true);

    current_.exposure_lines = std::clamp(exposure, ae_.exposure_min, ae_.exposure_max);
    current_.gain = std::clamp<int32_t>(
      static_cast<int32_t>(std::lround(16.0 * (1.0 + uvc_gain_slope_ * gain))),
      ae_.gain_min, ae_.gain_max);
    uvc_ae_ = true;
    camera_->setControl(V4L2_CID_EXPOSURE_ABSOLUTE, current_.exposure_lines);
    camera_->setControl(V4L2_CID_GAIN, uvcGain(current_.gain));
    uvc_requested_ = current_;
    uvc_used_ = current_;
    uvc_writer_ = std::thread([this]() {uvcWriterLoop();});
    exposure_pub_ = create_publisher<v4l2_trigger_camera::msg::CameraExposure>(
      "exposure", rclcpp::SensorDataQoS());
    RCLCPP_INFO(
      get_logger(), "closed-loop exposure (%s): exposure 1..%d x100us, gain 0..%d "
      "(%.2fx), black %.1f x gain, lands %.0f ms, settle %.0f ms, stamp=%s", ae_mode_.c_str(),
      ae_.exposure_max, uvc_gain_max_, ae_.gain_max / 16.0, ae_.black_level,
      uvc_write_latency_ms_, uvc_settle_ms_, mid_exposure_stamp_ ? "mid-exposure" : "trigger");
  }

  int32_t uvcGain(int32_t gain16) const
  {
    return std::clamp<int32_t>(
      static_cast<int32_t>(std::lround((gain16 / 16.0 - 1.0) / uvc_gain_slope_)), 0, uvc_gain_max_);
  }

  //  Hand a setting to the writer thread. `trigger_ns` is the trigger time of
  //  the frame metered for it. A UVC control write is a USB control transfer, measured at up to 20 ms
  //  on the rig: done inline it held back the frame being published.
  void writeUvcExposure(const ExposureSetting & s, int64_t trigger_ns)
  {
    const int32_t g = uvcGain(s.gain);
    if (s.exposure_lines == uvc_requested_.exposure_lines && g == uvcGain(uvc_requested_.gain)) {
      return;   // rounds to the controls already requested
    }
    uvc_requested_ = s;
    {
      std::lock_guard<std::mutex> lock(uvc_write_mutex_);
      uvc_pending_ = s;
    }
    uvc_write_cv_.notify_one();
    uvc_applied_.push_back({trigger_ns + static_cast<int64_t>(uvc_write_latency_ms_ * 1e6), s});
    uvc_next_meter_ns_ = trigger_ns + static_cast<int64_t>(uvc_settle_ms_ * 1e6);
  }

  //  Writes only the controls that changed. A newer request replaces one not
  //  yet written; the settle hold-off keeps that rare.
  void uvcWriterLoop()
  {
    int32_t written_exposure = -1, written_gain = -1;
    while (true) {
      ExposureSetting s;
      {
        std::unique_lock<std::mutex> lock(uvc_write_mutex_);
        uvc_write_cv_.wait(lock, [this] {return uvc_pending_.has_value() || !running_;});
        if (!running_) { return; }
        s = *uvc_pending_;
        uvc_pending_.reset();
      }
      const auto t0 = std::chrono::steady_clock::now();
      const int32_t g = uvcGain(s.gain);
      if (s.exposure_lines != written_exposure) {
        if (camera_->setControl(V4L2_CID_EXPOSURE_ABSOLUTE, s.exposure_lines)) {
          written_exposure = s.exposure_lines;
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "exposure write failed");
        }
      }
      if (g != written_gain) {
        if (camera_->setControl(V4L2_CID_GAIN, g)) {
          written_gain = g;
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "gain write failed");
        }
      }
      const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      std::lock_guard<std::mutex> lock(uvc_write_mutex_);
      uvc_writes_++;
      uvc_write_ms_max_ = std::max(uvc_write_ms_max_, ms);
    }
  }

  //  Meter the decoded frame, and step the controller once the last write has
  //  landed. `trigger_ns` is this frame's trigger time. Returns the setting the
  //  frame is taken to have used.
  ExposureSetting uvcExposureStep(
    int64_t trigger_ns, const uint8_t * gray, uint32_t w, uint32_t h,
    double & mean, double & saturated, double & metric)
  {
    while (!uvc_applied_.empty() && trigger_ns > uvc_applied_.front().first) {
      uvc_used_ = uvc_applied_.front().second;
      uvc_applied_.pop_front();
    }
    if (!gray) { return uvc_used_; }
    meterGray(
      gray, w, h, w, static_cast<uint32_t>(ae_meter_step_), mean, saturated, ae_.white_level);
    if (trigger_ns <= uvc_next_meter_ns_) { return uvc_used_; }
    const ExposureSetting next = ae_mode_ == "gradient" ?
      gradientExposureStep(
      ae_, grad_, gradient_metric_, current_, gray, w, h, w, mean, saturated, metric) :
      autoExposureStep(ae_, current_, mean, saturated);
    if (!(next == current_)) {
      current_ = next;
      writeUvcExposure(current_, trigger_ns);
    }
    return uvc_used_;
  }

  void applyExposure(const std::string & profile, int exposure, int gain)
  {
    //  Values come from the measured indoor sweep at 320x240; outdoor has not
    //  been characterised yet and is a starting guess, not a validated profile.
    int exp = exposure;
    bool automatic = false;

    if (profile == "manual_indoor") { exp = 10; } else if (profile == "manual_outdoor") {
      exp = 2;
    } else if (profile == "manual_custom" || profile == "closed_loop") {
      exp = exposure;
    } else if (profile == "auto_validation") {
      automatic = true;
    } else {
      RCLCPP_WARN(get_logger(), "unknown exposure_profile '%s', using manual_custom",
                  profile.c_str());
    }

    camera_->setControl(
      V4L2_CID_EXPOSURE_AUTO,
      automatic ? V4L2_EXPOSURE_APERTURE_PRIORITY : V4L2_EXPOSURE_MANUAL);
    if (!automatic) {
      camera_->setControl(V4L2_CID_EXPOSURE_ABSOLUTE, exp);
    }
    camera_->setControl(V4L2_CID_GAIN, gain);

    int32_t read_back = -1;
    camera_->getControl(V4L2_CID_EXPOSURE_ABSOLUTE, read_back);
    RCLCPP_INFO(
      get_logger(), "exposure_profile=%s exposure=%d (device reports %d) gain=%d",
      profile.c_str(), automatic ? -1 : exp, read_back, gain);
  }

  //  Match on time rather than arrival order, so a dropped frame cannot slide
  //  the stream permanently out of correspondence.
  //
  //  The frame lags its trigger by exposure + readout + USB transfer. That lag
  //  is not a constant of the system: it was measured at ~4.2 ms, and after the
  //  camera re-enumerated on a UPS switchover it settled near ~10 ms and wanders
  //  by a couple of ms between windows. A plain "most recent trigger at or
  //  before the frame" rule spends that entire lag out of its error budget, so
  //  a lag drifting toward half the trigger period would start mispairing.
  //
  //  So subtract a running estimate of the lag first and take the nearest
  //  trigger to the corrected time. That keeps the match centred whatever the
  //  lag happens to be, leaving the full +/- half period for jitter.
  //
  //  The CSI path passes `predicted_ms`: there the lag is known per frame from
  //  the exposure in use, and auto exposure moves it faster than the running
  //  estimate could follow.
  bool matchTrigger(
    const rclcpp::Time & frame_time, rtimulib_ros2::msg::CameraTrigger & out, double & delay_ms,
    double predicted_ms = -1.0)
  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    if (triggers_.empty()) { return false; }

    const double lag_ms = predicted_ms >= 0.0 ? predicted_ms : delay_estimate_ms_;
    const rclcpp::Time target =
      frame_time - rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(lag_ms * 1e6));

    bool found = false;
    double best = 0.0;
    for (auto it = triggers_.rbegin(); it != triggers_.rend(); ++it) {
      const rclcpp::Time t(it->header.stamp);
      const double diff = std::abs((target - t).seconds() * 1000.0);
      if (!found || diff < best) {
        best = diff;
        out = *it;
        found = true;
      } else {
        //  Triggers are ordered, so once the distance grows we are past the
        //  nearest one.
        break;
      }
    }
    if (!found) { return false; }

    delay_ms = (frame_time - rclcpp::Time(out.header.stamp)).seconds() * 1000.0;
    if (delay_ms < 0.0 || delay_ms > trigger_max_delay_ms_) { return false; }

    //  Seed from the first match so the estimate does not have to crawl up from
    //  zero, then track slowly.
    delay_estimate_ms_ =
      have_delay_estimate_ ? (0.99 * delay_estimate_ms_ + 0.01 * delay_ms) : delay_ms;
    have_delay_estimate_ = true;
    return true;
  }

  void reportStats()
  {
    if (window_delays_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "%lu frames, %lu dropped, %lu decode_err | no trigger matches in window",
        static_cast<unsigned long>(frames_), static_cast<unsigned long>(dropped_),
        static_cast<unsigned long>(decode_errors_));
      return;
    }

    std::vector<double> d = window_delays_;
    std::sort(d.begin(), d.end());
    const auto q = [&d](double f) {
        return d[std::min(d.size() - 1, static_cast<size_t>(f * d.size()))];
      };

    double sum = 0.0;
    for (double x : d) { sum += x; }

    //  A delay beyond half the trigger period means the nearest-preceding-trigger
    //  rule could latch onto the wrong pulse, so count those explicitly.
    size_t risky = 0;
    if (trigger_period_ms_ > 0.0) {
      for (double x : d) {
        if (std::abs(x - delay_estimate_ms_) > trigger_period_ms_ * 0.5) { risky++; }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "%lu frames, %lu dropped, unmatched %lu | delay ms: median %.2f mean %.2f "
      "p95 %.2f p99 %.2f max %.2f | lag est %.2f | residual>half-period: %lu/%lu",
      static_cast<unsigned long>(frames_), static_cast<unsigned long>(dropped_),
      static_cast<unsigned long>(unmatched_),
      q(0.5), sum / static_cast<double>(d.size()), q(0.95), q(0.99), d.back(),
      delay_estimate_ms_,
      static_cast<unsigned long>(risky), static_cast<unsigned long>(d.size()));

    window_delays_.clear();
  }

  //  Track the observed trigger cadence so the mismatch check above has a
  //  reference without needing the divisor configured here.
  void noteTriggerPeriod(const rclcpp::Time & t)
  {
    if (have_last_trigger_) {
      const double dt = (t - last_trigger_time_).seconds() * 1000.0;
      if (dt > 0.0 && dt < 1000.0) {
        trigger_period_ms_ =
          trigger_period_ms_ <= 0.0 ? dt : 0.98 * trigger_period_ms_ + 0.02 * dt;
      }
    }
    last_trigger_time_ = t;
    have_last_trigger_ = true;
  }

  void captureLoop()
  {
    while (rclcpp::ok() && running_) {
      V4L2Device::Frame frame;
      int r;
      try {
        r = camera_->wait(frame, 500);
      } catch (const std::exception & err) {
        //  A USB re-enumeration shows up here as ENODEV. Breaking out of the
        //  loop alone would leave the process alive with the topic advertised
        //  and nothing ever published, which looks like a hang rather than a
        //  fault. Shut the node down so a respawning launch restarts it and
        //  re-runs the full device setup against the current device node.
        RCLCPP_ERROR(get_logger(), "capture failed, shutting down to respawn: %s", err.what());
        running_ = false;
        rclcpp::shutdown();
        return;
      }
      if (r <= 0) {
        if (r == 0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "no frame for 500 ms; is the trigger running?");

          //  A USB glitch can leave the device open and enumerated but silently
          //  delivering nothing. That never raises ENODEV, so the error path
          //  above never fires and the node would warn forever with the topic
          //  advertised and dead - which is exactly how this failed in practice.
          //  Treat a sustained frame drought as fatal and let the launch respawn
          //  re-run the device setup, which is what actually clears it.
          const auto dry =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - last_frame_).count();
          if (no_frame_timeout_ > 0.0 && dry > no_frame_timeout_) {
            RCLCPP_ERROR(
              get_logger(),
              "no frames for %.1f s though the device is open; shutting down to respawn",
              dry);
            running_ = false;
            rclcpp::shutdown();
            return;
          }
        }
        continue;
      }
      last_frame_ = std::chrono::steady_clock::now();

      const auto ros_now = now();
      const auto v4l2_time = monotonicToRos(frame.timestamp, ros_now);

      if (!checked_jpeg_) {
        checked_jpeg_ = true;
        if (!hasHuffmanTable(frame.data, frame.size)) {
          RCLCPP_WARN(
            get_logger(),
            "MJPEG frames carry no Huffman table; consumers may fail to decode");
        } else {
          RCLCPP_INFO(get_logger(), "MJPEG carries a Huffman table, directly decodable");
        }
      }

      if (have_last_seq_ && frame.sequence != last_seq_ + 1) {
        dropped_ += (frame.sequence - last_seq_ - 1);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "camera reported dropped frames: sequence %u -> %u (%lu total)",
          last_seq_, frame.sequence, static_cast<unsigned long>(dropped_));
      }
      last_seq_ = frame.sequence;
      have_last_seq_ = true;

      //  Decode first: closed-loop exposure meters the decoded frame, and its
      //  write should go out as early as possible.
      unsigned rw = 0, rh = 0;
      bool decoded = false;
      if (publish_raw_ || uvc_ae_) {
        decoded = decodeGray(
          static_cast<const unsigned char *>(frame.data), frame.size,
          static_cast<unsigned>(raw_scale_denom_), gray_, rw, rh);
        if (!decoded && decode_errors_++ % 100 == 0) {
          RCLCPP_WARN(
            get_logger(),
            "MJPEG decode failed (%lu so far); /camera/image_raw will be silent "
            "while /camera/image/compressed keeps publishing",
            static_cast<unsigned long>(decode_errors_));
        }
      }
      rclcpp::Time stamp = v4l2_time;
      rtimulib_ros2::msg::CameraTrigger trig;
      double delay_ms = 0.0;
      const bool matched = use_trigger_stamp_ && matchTrigger(v4l2_time, trig, delay_ms);

      double mean = 0.0, saturated = 0.0, metric = 0.0;
      ExposureSetting used;
      if (uvc_ae_) {
        //  Unmatched, the trigger is estimated from the running lag.
        const int64_t trigger_ns = matched ?
          rclcpp::Time(trig.header.stamp).nanoseconds() :
          v4l2_time.nanoseconds() - static_cast<int64_t>(delay_estimate_ms_ * 1e6);
        used = uvcExposureStep(
          trigger_ns, decoded ? gray_.data() : nullptr, rw, rh, mean, saturated, metric);
      }

      if (matched) {
        //  The V4L2 timestamp is quantised to a 4 ms grid on this camera; the
        //  trigger edge is the better timebase, so prefer it when available.
        stamp = trig.header.stamp;
        if (uvc_ae_ && mid_exposure_stamp_) {
          stamp += rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(0.5 * used.exposure_lines * 100e3));
        }
        window_delays_.push_back(delay_ms);
        matched_++;
      } else if (use_trigger_stamp_) {
        unmatched_++;
      }

      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id_;
      msg->format = "jpeg";
      msg->data.assign(frame.data, frame.data + frame.size);
      image_pub_->publish(std::move(msg));

      if (publish_raw_ && decoded) {
        auto raw = std::make_unique<sensor_msgs::msg::Image>();
        raw->header.stamp = stamp;          // same trigger-exact stamp
        raw->header.frame_id = frame_id_;
        raw->height = rh;
        raw->width = rw;
        raw->encoding = "mono8";
        raw->is_bigendian = 0;
        raw->step = rw;
        raw->data.assign(gray_.begin(), gray_.end());
        raw_pub_->publish(std::move(raw));
      }

      if (uvc_ae_) {
        v4l2_trigger_camera::msg::CameraExposure ex;
        ex.header.stamp = stamp;
        ex.header.frame_id = frame_id_;
        ex.exposure_lines = static_cast<uint32_t>(used.exposure_lines);
        ex.exposure_us = static_cast<float>(used.exposure_lines * 100.0);
        ex.analogue_gain = static_cast<uint32_t>(uvcGain(used.gain));
        ex.exposure_measured = false;
        ex.auto_exposure = true;
        ex.mean_level = static_cast<float>(mean);
        ex.saturated_fraction = static_cast<float>(saturated);
        ex.gradient_metric = static_cast<float>(metric);
        exposure_pub_->publish(ex);
        window_mean_ += mean;
        window_n_++;
      }

      sensor_msgs::msg::CameraInfo info;
      info.header.stamp = stamp;
      info.header.frame_id = frame_id_;
      info.width = camera_->width();
      info.height = camera_->height();
      info_pub_->publish(info);

      camera_->queue(frame.index);

      if (++frames_ % stats_every_ == 0) {
        reportStats();
        if (uvc_ae_) {
          uint64_t writes = 0;
          double slowest = 0.0;
          {
            std::lock_guard<std::mutex> lock(uvc_write_mutex_);
            writes = uvc_writes_;
            slowest = uvc_write_ms_max_;
            uvc_writes_ = 0;
            uvc_write_ms_max_ = 0.0;
          }
          RCLCPP_INFO(
            get_logger(), "  exposure %d (%.1f ms) gain %d | mean %.1f sat %.2f%% | "
            "%lu writes, slowest %.2f ms (off the capture thread)", current_.exposure_lines,
            current_.exposure_lines * 0.1, uvcGain(current_.gain),
            window_n_ ? window_mean_ / window_n_ : 0.0, saturated * 100.0,
            static_cast<unsigned long>(writes), slowest);
          window_mean_ = 0.0;
          window_n_ = 0;
        }
      }
    }
  }

  std::unique_ptr<V4L2Device> camera_;

  //  CSI only.
  struct Write
  {
    rclcpp::Time done;
    ExposureSetting setting;
  };
  std::unique_ptr<CsiCamera> csi_;
  std::unique_ptr<JpegM2M> jpeg_;
  rclcpp::Publisher<v4l2_trigger_camera::msg::CameraExposure>::SharedPtr exposure_pub_;
  AutoExposureConfig ae_;
  std::string ae_mode_{"mean"};
  GradientConfig grad_;
  GradientMetric gradient_metric_;
  bool auto_exposure_{true};
  int64_t ae_meter_step_{4};
  ExposureSetting manual_;
  ExposureSetting current_;
  std::deque<Write> writes_;            // capture thread only
  std::vector<uint8_t> pixels_;         // cached copy of the current frame
  int32_t exposure_ceiling_{397};
  int32_t exposure_frame_limit_{397};   // frame length - 25: the ceiling never exceeds it
  int32_t vblank_base_{22};             // frame blanking outside low light
  int32_t vblank_max_{22};
  int32_t vblank_cur_{22};              // last written; follows long exposures
  double max_skip_rate_{0.005};
  uint64_t window_pulses_{0};
  uint64_t window_skips_{0};
  bool window_near_ceiling_{false};
  double csi_latency_offset_ms_{0.883};
  double csi_line_us_{15.94};
  double mid_exposure_offset_us_{0.0};
  uint64_t last_trigger_count_{0};
  bool have_last_trigger_count_{false};
  uint64_t skipped_pulses_{0};
  double window_mean_{0.0};
  uint64_t window_n_{0};

  //  UVC closed-loop exposure only.
  bool uvc_ae_{false};
  bool mid_exposure_stamp_{false};
  double uvc_gain_slope_{0.0189};
  int32_t uvc_gain_max_{100};
  double uvc_write_latency_ms_{42.0};
  double uvc_settle_ms_{60.0};
  int64_t uvc_next_meter_ns_{0};       // meter again after this trigger time
  ExposureSetting uvc_requested_;      // last setting handed to the writer
  ExposureSetting uvc_used_;
  std::deque<std::pair<int64_t, ExposureSetting>> uvc_applied_;   // lands after this trigger time
  std::thread uvc_writer_;
  std::mutex uvc_write_mutex_;          // guards the three below
  std::condition_variable uvc_write_cv_;
  std::optional<ExposureSetting> uvc_pending_;
  uint64_t uvc_writes_{0};
  double uvc_write_ms_max_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  bool publish_raw_{true};
  int raw_scale_denom_{2};
  std::vector<uint8_t> gray_;       // reused across frames; resize is a no-op
  uint64_t decode_errors_{0};
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<rtimulib_ros2::msg::CameraTrigger>::SharedPtr trigger_sub_;

  std::deque<rtimulib_ros2::msg::CameraTrigger> triggers_;
  std::mutex trigger_mutex_;

  std::string frame_id_;
  bool use_trigger_stamp_{true};
  double trigger_max_delay_ms_{50.0};
  double no_frame_timeout_{10.0};
  std::chrono::steady_clock::time_point last_frame_{};
  int64_t stats_every_{300};

  bool checked_jpeg_{false};
  bool have_last_seq_{false};
  uint32_t last_seq_{0};
  uint64_t frames_{0}, dropped_{0}, matched_{0}, unmatched_{0};
  std::vector<double> window_delays_;
  double trigger_period_ms_{0.0};
  double delay_estimate_ms_{0.0};
  bool have_delay_estimate_{false};
  rclcpp::Time last_trigger_time_;
  bool have_last_trigger_{false};

  std::thread worker_;
  std::atomic_bool running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CameraNode>());
  } catch (const std::exception & err) {
    RCLCPP_FATAL(rclcpp::get_logger("v4l2_trigger_camera"), "%s", err.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
