#include <algorithm>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/videodev2.h>
#include <time.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rtimulib_ros2/msg/camera_trigger.hpp>

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

    camera_ = std::make_unique<V4L2Device>(
      device, static_cast<uint32_t>(width), static_cast<uint32_t>(height),
      static_cast<uint32_t>(fps));

    applyExposure(exposure_profile, exposure, gain);

    //  On this camera exposure_dynamic_framerate is what puts the sensor into
    //  externally triggered capture; it is not a frame-rate hint.
    if (!camera_->setControl(V4L2_CID_EXPOSURE_AUTO_PRIORITY, trigger_mode ? 1 : 0)) {
      RCLCPP_WARN(get_logger(), "could not set exposure_dynamic_framerate (trigger mode)");
    }

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
      camera_->width(), camera_->height(), device.c_str(),
      trigger_mode ? "on" : "off", use_trigger_stamp_ ? "trigger" : "v4l2");

    worker_ = std::thread([this]() {captureLoop();});
  }

  ~CameraNode() override
  {
    running_ = false;
    if (worker_.joinable()) { worker_.join(); }
  }

private:
  void applyExposure(const std::string & profile, int exposure, int gain)
  {
    //  Values come from the measured indoor sweep at 320x240; outdoor has not
    //  been characterised yet and is a starting guess, not a validated profile.
    int exp = exposure;
    bool automatic = false;

    if (profile == "manual_indoor") { exp = 10; } else if (profile == "manual_outdoor") {
      exp = 2;
    } else if (profile == "manual_custom") {
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
  bool matchTrigger(
    const rclcpp::Time & frame_time, rtimulib_ros2::msg::CameraTrigger & out, double & delay_ms)
  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    if (triggers_.empty()) { return false; }

    const rclcpp::Time target =
      frame_time - rclcpp::Duration::from_nanoseconds(
      static_cast<int64_t>(delay_estimate_ms_ * 1e6));

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

      rclcpp::Time stamp = v4l2_time;
      rtimulib_ros2::msg::CameraTrigger trig;
      double delay_ms = 0.0;
      const bool matched = use_trigger_stamp_ && matchTrigger(v4l2_time, trig, delay_ms);
      if (matched) {
        //  The V4L2 timestamp is quantised to a 4 ms grid on this camera; the
        //  trigger edge is the better timebase, so prefer it when available.
        stamp = trig.header.stamp;
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

      if (publish_raw_) {
        unsigned rw = 0, rh = 0;
        if (decodeGray(
            static_cast<const unsigned char *>(frame.data), frame.size,
            static_cast<unsigned>(raw_scale_denom_), gray_, rw, rh))
        {
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
        } else {
          if (decode_errors_++ % 100 == 0) {
            RCLCPP_WARN(
              get_logger(),
              "MJPEG decode failed (%lu so far); /camera/image_raw will be silent "
              "while /camera/image/compressed keeps publishing",
              static_cast<unsigned long>(decode_errors_));
          }
        }
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
      }
    }
  }

  std::unique_ptr<V4L2Device> camera_;
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
