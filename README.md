# v4l2_trigger_camera

ROS 2 Humble node publishing MJPEG from a hardware-triggered global-shutter UVC
camera (Arducam OV9281, UC-844) as `sensor_msgs/msg/CompressedImage`.

```text
/camera/image/compressed   sensor_msgs/msg/CompressedImage, format "jpeg"
/camera/camera_info        sensor_msgs/msg/CameraInfo
/camera/trigger            subscribed, rtimulib_ros2/msg/CameraTrigger
```

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

## Known issues

- Manual exposure does not appear to take effect; the camera runs its own
  auto-exposure. This does not matter for feature-based VIO with histogram
  equalisation, but it means frame rate varies with scene brightness, and it may
  explain the wandering trigger-to-frame lag. Untested.
- At 1280x800 and 58.8 Hz, messages are ~57 kB and Fast-DDS loses a few percent
  of them to UDP fragmentation before they reach a subscriber, while the node
  itself reports 0 V4L2 drops. Shared-memory transport should avoid this.
