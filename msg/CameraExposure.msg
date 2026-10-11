# Exposure settings in effect for one image, published alongside it with the
# same header.stamp, so a consumer can pair them exactly.
#
# Only the CSI (UC-580) source fills this: the UVC camera runs its own exposure
# and does not report what it used.

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
