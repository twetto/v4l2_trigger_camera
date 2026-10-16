#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

//  Auto exposure for feature-tracking VIO, not for pleasant pictures.
//
//  What VIO needs from it, and what follows:
//  - Brightness of a tracked patch should barely change between consecutive
//    frames (KLT assumes brightness constancy). So the per-frame change in
//    exposure x gain is capped (max_step_ratio) and small errors are ignored
//    (deadband) rather than chased.
//  - Exposure before gain: gain amplifies noise, exposure does not, and on the
//    trigger the frame's timing is known per frame anyway (mid-exposure stamp).
//    Gain only rises once exposure is at its ceiling.
//  - Clipped highlights lose features outright, so saturation overrides the
//    brightness target and pushes exposure down.
//
//  On the rig trigger a write takes effect on the very next frame (measured), so
//  this is a one-frame-delay loop: meter frame k, set frame k+1. A camera with
//  more delay (UVC: 3 frames) must hold off metering until its write has landed;
//  the caller does that, and these steps are unchanged.
//
//  Brightness above black is taken as proportional to exposure x gain. That was
//  measured for exposure (+1% over 200..1797 lines) and approximately for gain
//  (doubling gain about doubled the signal); the controller only needs it to be
//  monotonic and roughly proportional, since it closes the loop every frame.

struct AutoExposureConfig
{
  double target_level{100.0};       // desired metered mean, 0-255
  double black_level{16.0};         // sensor output at zero light
  double deadband{0.06};            // |log error| below this: hold
  double correction{0.5};           // fraction of the log error corrected per frame
  double max_step_ratio{1.12};      // exposure x gain may change at most 12% per frame
  //  Above this fraction of clipped pixels, darken; brighten only below
  //  saturation_resume. Lamps and specular hot spots clip at any exposure: on
  //  the rig they were 1.0-1.4% of the frame, and a 1%/2% band left the image at
  //  mean 25-34 of 255 under them. A dark frame costs VIO everywhere, a clipped
  //  lamp only where the lamp is.
  double saturation_limit{0.05};
  double saturation_resume{0.025};
  //  Clipping may cost the rest of the frame only down to this mean of the
  //  unclipped pixels (0-255). Without it, a ceiling light filling 4.6% of an
  //  upward view drove exposure to 23 lines: everything but the lamp sat at
  //  black (unclipped mean ~15), and clipping inside the hysteresis band then
  //  forbade brightening for good.
  double saturation_floor_level{60.0};
  int32_t exposure_min{1};
  int32_t exposure_max{397};        // lines; the caller keeps this at the trigger-safe ceiling
  int32_t gain_min{16};             // 16 = 1x on this sensor
  int32_t gain_max{255};
  //  Low light: past exposure_max and long_exposure_gain, exposure grows again
  //  up to long_exposure_max, and the sensor skips the trigger pulses it cannot
  //  follow (measured: frames stay whole and matched; 59 -> 29.5 -> ~20 Hz as
  //  exposure passes ~6.3, ~10 and ~17 ms). Frame rate is traded for light, gain
  //  up to long_exposure_gain is used first because it costs no blur. 0 (or
  //  anything <= exposure_max) turns it off.
  int32_t long_exposure_max{0};
  int32_t long_exposure_gain{64};   // 4x
  //  The output's black level is multiplied by gain along with the signal. True
  //  of the Arducam UVC camera, whose bridge applies gain after the pedestal:
  //  the gain-g / gain-0 mean ratio was the same at two exposures (1.94 / 1.92
  //  at g=50), while ratios of mean-above-black differed (3.77 / 2.83). False on
  //  CSI, where the sensor's own analogue gain leaves black where it is.
  //  target_level and the floor levels stay defined at 1x: they are compared as
  //  signal above black, black_level, at whatever gain is in use.
  bool black_scales_with_gain{false};
  //  Pixels at or above this count as clipped. 255 on CSI. The Arducam UVC
  //  camera clips at 244 at gain 0 (99.6% of an overexposed frame read exactly
  //  244, none 255; outdoor bag vio_live_20261016-065308), so counting only 255
  //  saw no clipping at all and the image stayed white.
  int32_t white_level{255};
};

struct ExposureSetting
{
  int32_t exposure_lines{300};
  int32_t gain{32};

  bool operator==(const ExposureSetting & o) const
  {
    return exposure_lines == o.exposure_lines && gain == o.gain;
  }
};

//  Black level of a frame taken at `gain`.
inline double blackAt(const AutoExposureConfig & c, int32_t gain)
{
  return c.black_scales_with_gain ? c.black_level * gain / 16.0 : c.black_level;
}

//  Exposure in line units times gain in units of 1x.
inline double exposureProduct(const ExposureSetting & s)
{
  return static_cast<double>(s.exposure_lines) * static_cast<double>(s.gain) / 16.0;
}

//  Exposure ceiling including the low-light extension.
inline int32_t longExposureMax(const AutoExposureConfig & c)
{
  return std::max(c.exposure_max, c.long_exposure_max);
}

//  Split a desired exposure x gain into the two controls, in four stages:
//  exposure up to exposure_max, gain up to long_exposure_gain, exposure up to
//  long_exposure_max (frame rate drops), then the rest of the gain. With the
//  extension off the third stage is empty: exposure first, then gain. Both are
//  integers, so rounding is done towards `from`: rounding away from it could
//  push one frame's change past max_step_ratio (seen in simulation: 13.8%
//  against a 12% cap).
inline ExposureSetting splitExposure(
  const AutoExposureConfig & c, double product, const ExposureSetting & from)
{
  ExposureSetting s;
  const int32_t e_long = longExposureMax(c);
  const int32_t g_long = std::clamp(c.long_exposure_gain, c.gain_min, c.gain_max);
  const double lo = static_cast<double>(c.exposure_min) * c.gain_min / 16.0;
  const double hi = static_cast<double>(e_long) * c.gain_max / 16.0;
  product = std::clamp(product, lo, hi);
  const bool up = product >= exposureProduct(from);
  const auto toward = [up](double v) {return static_cast<int32_t>(up ? std::floor(v) : std::ceil(v));};

  const double exposure = product <= static_cast<double>(c.exposure_max) * g_long / 16.0 ?
    std::min<double>(product / (c.gain_min / 16.0), c.exposure_max) :
    std::min<double>(product / (g_long / 16.0), e_long);
  s.exposure_lines = std::clamp<int32_t>(toward(exposure), c.exposure_min, e_long);
  if (s.exposure_lines > c.exposure_max && s.exposure_lines < e_long) {
    //  Third stage: gain is pinned, so it never reads as out of place below.
    s.gain = g_long;
    return s;
  }
  //  Gain makes up whatever exposure could not.
  const double gain = product / static_cast<double>(s.exposure_lines) * 16.0;
  s.gain = std::clamp<int32_t>(toward(gain), c.gain_min, c.gain_max);
  //  A darkening step smaller than one exposure unit rounds back to `from`. In
  //  coarse units that is a dead end: on the UVC camera (100 us units) every
  //  12% step down from exposure 9 or less rounded to no change, and a frame
  //  99.7% clipped held exposure 9 for 47 s (bag vio_live_20261016-073136).
  //  Darkening is what the clip override needs, so take one unit. That exceeds
  //  max_step_ratio below exposure ~9 (2 -> 1 is 50%). Brightening is left to
  //  round: forcing both ways would let a deadband-sized error cycle +-1.
  if (!up && s == from && from.exposure_lines > c.exposure_min &&
    product < exposureProduct(from))
  {
    s.exposure_lines = from.exposure_lines - 1;
  }
  return s;
}

//  Mean of the pixels below 255, from the full mean (clipped counted as 255)
//  and the clipped fraction. Exact for the metered grid, no extra pass.
inline double unclippedMean(double mean, double saturated, double white = 255.0)
{
  if (saturated >= 1.0) {return white;}
  return (mean - white * saturated) / (1.0 - saturated);
}

//  Where clipping stands against the floor on the rest of the frame: `starved`
//  when the unclipped pixels are already below saturation_floor_level, and
//  `may_darken` when one darkening step would still leave them at or above it.
//  Darkening needs the post-step value to clear the floor, and brightening from
//  below stops within one step of it, so the two cannot alternate.
struct ClipFloor
{
  bool starved;
  bool may_darken;
  double log_to_floor;              // log(floor / unclipped), signal above black
};

inline ClipFloor clipFloor(
  const AutoExposureConfig & c, double mean, double saturated, int32_t gain)
{
  const double floor_signal = std::max(c.saturation_floor_level - c.black_level, 1.0);
  const double clear_signal =
    std::max(unclippedMean(mean, saturated, c.white_level) - blackAt(c, gain), 1.0);
  return {clear_signal < floor_signal, clear_signal / c.max_step_ratio >= floor_signal,
    std::log(floor_signal / clear_signal)};
}

//  One control step. `mean` is the metered mean (clipped pixels count as 255),
//  `saturated` the fraction of metered pixels at 255.
inline ExposureSetting autoExposureStep(
  const AutoExposureConfig & c, const ExposureSetting & current, double mean, double saturated)
{
  //  The ceiling can drop under the current exposure (a skipped pulse lowers
  //  it). Move the excess into gain at once, whatever the brightness error: a
  //  hold here kept exposure at 397 lines after the ceiling fell to 377, and the
  //  sensor went on skipping pulses.
  //  Past exposure_max is only right in the low-light stages, at gain >=
  //  long_exposure_gain and within long_exposure_max.
  if (current.exposure_lines > c.exposure_max &&
    (current.exposure_lines > longExposureMax(c) ||
    current.gain < std::clamp(c.long_exposure_gain, c.gain_min, c.gain_max)))
  {
    return splitExposure(c, exposureProduct(current), current);
  }

  const double signal = std::max(mean - blackAt(c, current.gain), 1.0);
  const double wanted = std::max(c.target_level - c.black_level, 1.0);
  double log_err = std::log(wanted / signal);

  //  Clipping hides how bright the scene really is, so the mean understates it;
  //  never brighten while too much is clipped, and darken at least one step.
  //
  //  Brightening is only allowed again once clipping has fallen to
  //  saturation_resume. A single threshold made a limit cycle on the rig:
  //  clipping crossed 2% each way and the setting changed on 98.9% of frames
  //  (-4%, -4%, +9%, repeating), a frame-to-frame flicker that defeats KLT.
  //
  //  Both give way to the floor on the unclipped pixels (clipFloor): a light
  //  source clips at any exposure, and chasing it only blacks out the rest.
  const ClipFloor f = clipFloor(c, mean, saturated, current.gain);
  if (saturated > c.saturation_limit && f.may_darken) {
    log_err = std::min(log_err, -std::log(c.max_step_ratio));
  } else if (f.starved) {
    log_err = std::max(log_err, f.log_to_floor);
    if (log_err < c.deadband) {return current;}
  } else if (std::abs(log_err) < c.deadband || saturated > c.saturation_limit ||
    (log_err > 0.0 && saturated > c.saturation_resume))
  {
    return current;
  }

  const double max_log = std::log(c.max_step_ratio);
  const double step = std::clamp(c.correction * log_err, -max_log, max_log);
  return splitExposure(c, exposureProduct(current) * std::exp(step), current);
}

//  Metering on a subsampled grid: every `step`-th pixel in both directions.
//  Uniform weighting; VIO cares about the whole field, not a centre subject.
//  `saturated` is the fraction at or above `white`.
inline void meterGray(
  const uint8_t * data, uint32_t width, uint32_t height, uint32_t stride, uint32_t step,
  double & mean, double & saturated, int32_t white = 255)
{
  uint64_t sum = 0, n = 0, sat = 0;
  for (uint32_t y = 0; y < height; y += step) {
    const uint8_t * row = data + static_cast<size_t>(y) * stride;
    for (uint32_t x = 0; x < width; x += step) {
      const uint8_t v = row[x];
      sum += v;
      sat += (v >= white);
      n++;
    }
  }
  mean = n ? static_cast<double>(sum) / static_cast<double>(n) : 0.0;
  saturated = n ? static_cast<double>(sat) / static_cast<double>(n) : 0.0;
}

//  ---- Gradient mode ----------------------------------------------------------
//
//  After Zhang, Forster and Scaramuzza, "Active Exposure Control for Robust
//  Visual Odometry in HDR Environments" (ICRA 2017): choose the exposure that
//  maximises a soft percentile of the squared gradient magnitudes, eqs. (4),
//  (9), (10), with p = 0.7-0.8 and k = 5 as they recommend.
//
//  One change for this sensor. They step along the metric's derivative, which
//  they take through the camera response function; near white its curvature is
//  what makes the derivative foresee clipping. The OV9281 output is raw and
//  linear up to a hard clip, where that derivative is 2G/dt for every unclipped
//  pixel -- always "brighter", never anticipating the clip. But a linear sensor
//  lets the image at another exposure be predicted outright,
//      I' = black + r (I - black), clipped at 255,
//  so the metric is evaluated on the predicted images for exposure x {1/s, 1, s}
//  and the step goes to the best. Pixels already at 255 are kept at 255 when
//  darkening: what they hide is unknown, so darkening earns no credit for it.
//  That is why the saturation override is kept, in the role of the fixed
//  "overexposed" derivative in their Section VI-A3.
//
//  Gain is not optimised. Analogue gain amplifies the noise with the signal, so
//  a gradient metric would always reward it; Zhang et al. also leave gain to a
//  threshold heuristic. Here gain only rises when exposure is at its ceiling and
//  the image is dark enough to lose detail to quantisation, and falls back when
//  there is room.

struct GradientConfig
{
  double percentile{0.75};          // p in eq. (10)
  int k{5};                         // k in eq. (10)
  uint32_t step{2};                 // sample every step-th pixel in x and y
  double min_improvement{1.01};     // move only if the metric gains 1% or more
  double gain_floor_level{60.0};    // below this mean, at the exposure ceiling, raise gain
};

class GradientMetric
{
public:
  //  Gradient magnitude bins: |dI/dx| + |dI/dy| range, central differences.
  static constexpr int kBins = 724;

  //  Soft-percentile weights depend only on the sample count, so the cumulative
  //  weight table is built once per image size.
  void prepare(uint32_t width, uint32_t height, const GradientConfig & g)
  {
    const uint32_t nx = width > 2 ? (width - 2 + g.step - 1) / g.step : 0;
    const uint32_t ny = height > 2 ? (height - 2 + g.step - 1) / g.step : 0;
    const size_t s = static_cast<size_t>(nx) * ny;
    if (s == samples_ && g.percentile == p_ && g.k == k_) { return; }
    samples_ = s;
    p_ = g.percentile;
    k_ = g.k;
    cum_.assign(s + 1, 0.0);
    const double ps = std::floor(p_ * static_cast<double>(s));
    double total = 0.0;
    std::vector<double> w(s);
    for (size_t i = 0; i < s; i++) {
      const double x = static_cast<double>(i);
      const double v = x <= ps ?
        std::sin(M_PI / (2.0 * std::max(ps, 1.0)) * x) :
        std::sin(M_PI / 2.0 - M_PI / 2.0 * (x - ps) / std::max(static_cast<double>(s) - ps, 1.0));
      w[i] = std::pow(std::max(v, 0.0), k_);
      total += w[i];
    }
    for (size_t i = 0; i < s; i++) { cum_[i + 1] = cum_[i] + w[i] / total; }
  }

  //  M_softperc of the image after the pixel map `lut`, eq. (9): gradients
  //  sorted ascending, weighted by rank. Sorting is replaced by a histogram of
  //  gradient magnitude; each bin's mean G takes the weight of the ranks it spans.
  double evaluate(
    const uint8_t * data, uint32_t width, uint32_t height, uint32_t stride, uint32_t step,
    const uint8_t * lut)
  {
    std::fill(count_, count_ + kBins, 0u);
    std::fill(sum_, sum_ + kBins, 0.0);
    for (uint32_t y = 1; y + 1 < height; y += step) {
      const uint8_t * up = data + static_cast<size_t>(y - 1) * stride;
      const uint8_t * row = data + static_cast<size_t>(y) * stride;
      const uint8_t * dn = data + static_cast<size_t>(y + 1) * stride;
      for (uint32_t x = 1; x + 1 < width; x += step) {
        const int gx = static_cast<int>(lut[row[x + 1]]) - static_cast<int>(lut[row[x - 1]]);
        const int gy = static_cast<int>(lut[dn[x]]) - static_cast<int>(lut[up[x]]);
        const int bin = std::min(kBins - 1, std::abs(gx) + std::abs(gy));
        count_[bin]++;
        sum_[bin] += static_cast<double>(gx * gx + gy * gy);
      }
    }
    double m = 0.0;
    size_t rank = 0;
    for (int b = 0; b < kBins; b++) {
      if (!count_[b]) { continue; }
      const size_t end = std::min(rank + count_[b], samples_);
      m += sum_[b] / count_[b] * (cum_[end] - cum_[std::min(rank, samples_)]);
      rank += count_[b];
    }
    return m;
  }

private:
  size_t samples_{0};
  double p_{-1.0};
  int k_{-1};
  std::vector<double> cum_;
  uint32_t count_[kBins] {};
  double sum_[kBins] {};
};

//  The image this sensor would give at exposure x r: linear above black, clipped
//  at `white`, and anything already clipped stays clipped. Clipped pixels map to
//  255, so predictedSaturation can count them whatever the white level.
inline void exposureLut(double r, double black, uint8_t lut[256], int32_t white = 255)
{
  for (int v = 0; v < 256; v++) {
    if (v >= white) { lut[v] = 255; continue; }
    const double p = black + r * (static_cast<double>(v) - black);
    lut[v] = p >= white ? 255 : static_cast<uint8_t>(std::clamp(std::lround(p), 0L, 255L));
  }
}

//  Fraction of sampled pixels that would read 255 after the pixel map `lut`.
inline double predictedSaturation(
  const uint8_t * data, uint32_t width, uint32_t height, uint32_t stride, uint32_t step,
  const uint8_t * lut)
{
  uint64_t n = 0, sat = 0;
  for (uint32_t y = 0; y < height; y += step) {
    const uint8_t * row = data + static_cast<size_t>(y) * stride;
    for (uint32_t x = 0; x < width; x += step) {
      sat += lut[row[x]] == 255;
      n++;
    }
  }
  return n ? static_cast<double>(sat) / static_cast<double>(n) : 0.0;
}

//  One gradient-mode step. `metric` is filled with M_softperc of the frame as
//  captured, for logging.
inline ExposureSetting gradientExposureStep(
  const AutoExposureConfig & config, const GradientConfig & g, GradientMetric & gm,
  const ExposureSetting & current, const uint8_t * data, uint32_t width, uint32_t height,
  uint32_t stride, double mean, double saturated, double & metric)
{
  //  The low-light extension is mean mode only for now; here exposure stays
  //  within the full-rate ceiling as before.
  AutoExposureConfig c = config;
  c.long_exposure_max = 0;
  gm.prepare(width, height, g);
  uint8_t lut[3][256];
  const double s = c.max_step_ratio;
  //  The exposures a step down or up would actually set. Integer controls
  //  round, and a step that rounds to no change takes one unit, so in coarse
  //  units the real ratio can be far from s: 1 -> 2 is 2x. The metric and the
  //  clipping check are evaluated there. Judged at s instead, the UVC camera
  //  cycled 1 <-> 2 every step in simulation, clipping 0% <-> 7.6%.
  const auto candidate = [&](bool up) {
    const double e = current.exposure_lines * (up ? s : 1.0 / s);
    int32_t n = static_cast<int32_t>(up ? std::floor(e) : std::ceil(e));
    if (n == current.exposure_lines) { n += up ? 1 : -1; }
    return std::clamp(n, c.exposure_min, c.exposure_max);
  };
  const int32_t steps[3] = {candidate(false), current.exposure_lines, candidate(true)};
  double ratios[3];
  for (int i = 0; i < 3; i++) {
    ratios[i] = static_cast<double>(steps[i]) / std::max(current.exposure_lines, 1);
  }
  double m[3];
  for (int i = 0; i < 3; i++) {
    exposureLut(ratios[i], blackAt(c, current.gain), lut[i], c.white_level);
    m[i] = gm.evaluate(data, width, height, stride, g.step, lut[i]);
  }
  metric = m[1];

  if (current.exposure_lines > c.exposure_max) {
    return splitExposure(c, exposureProduct(current), current);
  }
  //  Too much clipped: darken, whatever the metric says (see above), unless
  //  that would take the unclipped pixels below the floor.
  if (saturated > c.saturation_limit && clipFloor(c, mean, saturated, current.gain).may_darken) {
    return splitExposure(c, exposureProduct(current) / s, current);
  }

  ExposureSetting next = current;
  int best = 1;
  //  Brightening is refused if the predicted clipping would pass
  //  saturation_resume, as for gain below. Without it the step up clipped past
  //  saturation_limit, the override took it back, and the setting alternated
  //  every step: exposure 14 <-> 15 on the UVC camera, clipping 3.2% <-> 7.6%,
  //  in a closed-loop simulation on an outdoor frame.
  const bool can_up = steps[2] > current.exposure_lines;
  const bool can_down = steps[0] < current.exposure_lines;
  if (can_up && m[2] > m[best] * g.min_improvement &&
    predictedSaturation(data, width, height, stride, g.step, lut[2]) <= c.saturation_resume)
  {
    best = 2;
  }
  if (can_down && m[0] > m[best] * g.min_improvement) { best = 0; }
  if (best != 1) {
    next.exposure_lines = steps[best];
    return next;
  }

  //  Exposure is where the metric wants it. Gain only guards against a frame
  //  too dark to keep detail above quantisation, and is shed when not needed.
  //
  //  Raising it is refused if the predicted clipping at the higher gain would
  //  pass saturation_resume. Without that check, under lamps in a dark room the
  //  raise pushed clipping over saturation_limit, the override pulled it back,
  //  and the setting changed on 98% of frames.
  const double black = blackAt(c, current.gain);
  if (current.exposure_lines >= c.exposure_max &&
    mean - black < g.gain_floor_level - c.black_level && current.gain < c.gain_max)
  {
    const int32_t higher = std::min<int32_t>(
      c.gain_max, std::max<int32_t>(current.gain + 1, static_cast<int32_t>(std::floor(current.gain * s))));
    uint8_t up_lut[256];
    //  Gain that also scales black scales every pixel: a map about zero.
    exposureLut(
      static_cast<double>(higher) / current.gain, c.black_scales_with_gain ? 0.0 : c.black_level,
      up_lut, c.white_level);
    if (predictedSaturation(data, width, height, stride, g.step, up_lut) <= c.saturation_resume) {
      next.gain = higher;
    }
  } else if (current.gain > c.gain_min) {
    //  Shed gain while one step down still leaves the frame 10% above the
    //  floor. A fixed band (shed above 1.5x the floor) stranded gain at 183 in
    //  simulation once a dim spell had raised it: the mean sat just inside.
    const int32_t lower = std::max<int32_t>(
      c.gain_min, std::min<int32_t>(current.gain - 1, static_cast<int32_t>(std::ceil(current.gain / s))));
    const double predicted = c.black_level + (mean - black) * lower / current.gain;
    if (predicted > 1.1 * g.gain_floor_level) { next.gain = lower; }
  }
  return next;
}
