# v4l2_trigger_camera

ROS 2 Humble node publishing JPEG from a hardware-triggered global-shutter
OV9281 as `sensor_msgs/msg/CompressedImage`. Two sources, picked at start-up by
`camera_source` (`auto`, `uvc`, `csi`):

- **UVC**: the Arducam UC-844 USB camera, camera-native MJPEG passed through.
- **CSI**: the Arducam UC-580 on the Pi camera connector (linux-rpi with the
  uc580 overlay; see `kernel/` in duckiebot-installation-guide). Greyscale from
  Unicam, encoded on the VideoCore, with the node's own auto exposure.

`auto` takes CSI when a `unicam` media device with an `ov9281` sensor exists,
else UVC when `device` is a `uvcvideo` device, and refuses both at once: each
needs its own calibration downstream.

```text
/camera/image/compressed   sensor_msgs/msg/CompressedImage, format "jpeg"
/camera/image_raw          sensor_msgs/msg/Image, mono8 (publish_raw)
/camera/camera_info        sensor_msgs/msg/CameraInfo (uncalibrated)
/camera/exposure           v4l2_trigger_camera/msg/CameraExposure (CSI only)
/camera/trigger            subscribed, rtimulib_ros2/msg/CameraTrigger
```

## CSI source

**Stamps are mid-exposure.** Unicam timestamps frame start, which follows the
trigger by `0.883 ms + 15.94 us x exposure lines` (measured, std ~1.4 us). The
node matches each frame to its trigger on that predicted delay, reads the
exposure the frame really used back from the measured delay, and stamps
`trigger + mid_exposure_offset_us + exposure / 2`. How the 0.883 ms splits
before and after exposure is not known; a constant there is what the
camera-IMU time-offset calibration absorbs, while exposure / 2 changes per frame
under auto exposure and is applied here.

**Exposure and gain are published per frame**, with the same stamp as the
image, and `exposure_measured` says whether the exposure came from the frame's
own timing. A write lands on the very next triggered frame (measured), and the
next pulse can be ~2 ms after a frame is dequeued, so the node meters and writes
before encoding. A write racing a pulse can split (seen: new gain with old
exposure); the timing still gives the right exposure, gain is the best guess.

**Auto exposure is for KLT, not for looks** (`auto_exposure.hpp`). Two modes,
`ae_mode`:

- `mean` (default) holds the metered mean at `ae_target_level` (100). Exposure x
  gain changes at most 12% per frame and errors inside a 6% deadband are held.
- `gradient` maximises the soft-percentile gradient metric of Zhang, Forster
  and Scaramuzza (ICRA 2017). The sensor is linear, so instead of their response
  curve the node scores the frame as it would look at exposure x1/1.12, x1 and
  x1.12 and steps to the best; gain only keeps the mean above
  `ae_gain_floor_level`. It costs about twice the CPU of `mean` (23.6% against
  11.5% of one core) and has not been shown to track better, so it is not the
  default.

**Clipping is handled with a band and a floor**, in both modes. Above 5% clipped
the controller darkens, and it brightens again only below 2.5%: with a single
threshold the setting changed on 98.9% of frames (-4%, -4%, +9%, repeating).
A lamp or ceiling light clips at any exposure, though, and chasing it once took
an upward view to 23 lines with everything but the lamps at black level (mean
of the unclipped pixels ~15). So darkening for clipping stops where the
unclipped pixels would fall below `ae_saturation_floor_level` (60), and below
that floor the controller brightens whatever the clipping. Under ceiling lights
it now settles at an unclipped mean of 60-62.

**Low light trades frame rate for exposure.** In order, the controller uses
exposure up to the full-rate ceiling, gain up to `ae_long_exposure_gain` (4x,
no blur), exposure up to `ae_long_exposure_max_ms` (33 ms, a motion-blur cap),
then the rest of the gain. The frame length follows the exposure, and the
sensor skips the pulses it cannot follow. Measured in trigger mode: frames stay
whole and matched (0 unmatched), the delay model holds to 0.01 ms up to 24 ms,
and the rate steps 59 -> 29.5 -> ~20 -> ~15 Hz as exposure passes ~6, ~10 and
~17 ms. On a 5-minute walk it spent 3239 of 12157 frames past the full-rate
ceiling and came back to 59 Hz afterwards. The cap is also kept below
`trigger_max_delay_ms` so frames stay matchable. `gradient` mode does not use
this yet.

**The full-rate exposure ceiling is learned.** It starts at the frame length
minus 25 lines (397 at 640x400 with minimum blanking; the driver keeps reporting
a stale 1797). The rig's pulses are irregular -- some come 10-13 ms after the
previous one instead of 16.9 -- so a small skip rate is accepted: per window of
600 pulses the ceiling drops 5% when more than `ae_max_skip_rate` (0.5%) were
skipped with exposure near it, and rises 2% after a window with none. An
instant ratchet had chased each rare early pulse, 397 -> 222 lines in 90 s.
Skips during long exposures are the intended trade and are not counted.

**JPEG on the VideoCore** (`jpeg_m2m.hpp`): about 2 ms a frame, with working
`jpeg_quality` (7.2 / 14.3 / 62 KiB at 10 / 50 / 95, 640x400). GStreamer's
v4l2jpegenc ignores the quality control; driving the device directly does not.

**The frame is copied out of the capture buffer first.** Unicam's mmap buffers
read as uncached: the gradient metric took 35 ms on the buffer and 1.8 ms on a
copy, and the copy costs 0.42 ms. With it the whole node measured 11.5% of one
core at 640x400 and 59 Hz in `mean` mode (15-21% before).

**A bent ribbon corrupts rows.** With the UC-580 under the rig, 15-21% of
frames had single rows of noise starting mid-row, depending only on the angle
of the camera ribbon (reseating did not help; disk traffic made no
difference). At a good angle, held with a zip tie, 0 of 1611 frames. Unicam
reports no CSI errors, so check images, not the kernel log.

**640x400 is the default** because it is the only mode that follows every 59 Hz
pulse on one CSI lane; it is a 2x2 binned image of the full field of view.

## Design notes

**No decode.** Camera-native JPEG bytes go straight into `CompressedImage`. That
is the whole point of a custom node over `v4l2_camera`: at 1280x800 and 58.8 Hz
the node costs about 1.5% of one core because it never touches the pixels.
The payload was checked to carry a Huffman table, so generic consumers can
decode it; the node warns on the first frame if that ever stops being true.

**Setup order matters.** S_FMT, then S_PARM, then controls, then REQBUFS, then
STREAMON. Reordering these was observed to leave triggered capture not working,
and `v4l2-ctl` does not configure this camera reliably.

**Frames are stamped from the trigger, not from V4L2.** The V4L2 buffer
timestamp on this camera is quantised to a 4 ms grid, while the GPIO trigger
edge has 0.70 ms jitter. The V4L2 timestamp is used only to find the matching
trigger; the stamp published is the trigger's. Verified: 1026 of 1027 published
image stamps were byte-exact matches to a trigger edge.

**Matching compensates for a lag that moves.** The frame lags its trigger by
exposure plus readout plus transfer, and that lag is not a constant of the
system: it has been measured at 4.2, 10.4, 8.7 and 7.6 ms with no configuration
change and no environmental explanation (same USB path, same link speed, no
undervoltage). Matching therefore subtracts a running estimate of the lag and
takes the nearest trigger, so the match stays centred whatever the lag is. A
plain "most recent trigger at or before the frame" rule spends the whole lag out
of its +/- half period budget and would start mispairing silently.

**Device loss is fatal on purpose.** A USB re-enumeration surfaces as ENODEV.
Rather than sit there with the topic advertised and nothing published, the node
exits so a respawning launch restarts it. Pair with a udev symlink, because
re-enumeration renumbers `/dev/video*`.

**A frame drought is fatal too.** ENODEV is not the only way this breaks. A USB
glitch can leave the device open and enumerated while it silently delivers
nothing: `poll()` just times out, no error is ever raised, and the node would
warn forever with the topic advertised and dead. That happened in practice after
a run of USB re-enumerations, and only a manual restart cleared it. So
`no_frame_timeout_sec`, default 10 s, treats a sustained drought as fatal and
lets the respawn re-run the device setup, which does clear it. The clock starts
at stream start rather than at the first frame, because the observed failure was
a node that opened the device cleanly and then never received anything at all.

## Known issues

- Manual exposure does not appear to take effect; the camera runs its own
  auto-exposure. This does not matter for feature-based VIO with histogram
  equalisation, but it means frame rate varies with scene brightness, and it may
  explain the wandering trigger-to-frame lag. Untested.
- At 1280x800 and 58.8 Hz, messages are ~57 kB and Fast-DDS loses a few percent
  of them to UDP fragmentation before they reach a subscriber, while the node
  itself reports 0 V4L2 drops. Shared-memory transport should avoid this.
