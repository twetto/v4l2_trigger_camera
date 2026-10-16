# Exposure settings in effect for one image, published alongside it with the
# same header.stamp, so a consumer can pair them exactly.
#
# Published by the CSI (UC-580) source, and by the UVC source with
# exposure_profile=closed_loop. For UVC, exposure_lines is the control value in
# 100 us units, analogue_gain the UVC gain control (0-100, 0 = 1x), and both are
# the values written, assumed to land on the first frame triggered 42 ms or more
# after the frame metered for them (not measured per frame).

std_msgs/Header header

float32 exposure_us        # exposure time actually used by this frame
uint32 exposure_lines      # same, in sensor lines
uint32 analogue_gain       # sensor analogue_gain control value (16 = 1x)
bool exposure_measured     # true: exposure_lines inferred from this frame's own
                           # trigger-to-frame timing; false: the value last
                           # written, which a write racing the trigger can miss
bool auto_exposure         # true when the node's auto exposure chose these
float32 mean_level         # metered mean, 0-255 (clipped pixels count as 255)
float32 saturated_fraction # fraction of metered pixels at 255
float32 gradient_metric    # soft-percentile gradient metric of this frame (gradient mode; 0 otherwise)
