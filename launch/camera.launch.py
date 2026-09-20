from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Full sensor resolution by default. The OV9281 is 1280x800 native, and
    # 320x240 is neither that aspect ratio nor a clean divisor of it, so the
    # smaller modes are rescaled and hide real detail. MJPEG keeps the bandwidth
    # reasonable: roughly 40 kB a frame, about 1.2 MB/s at the 29.4 Hz trigger.
    args = [
        DeclareLaunchArgument("device", default_value="/dev/arducam-ov9281"),
        DeclareLaunchArgument("width", default_value="1280"),
        DeclareLaunchArgument("height", default_value="800"),
        DeclareLaunchArgument("fps", default_value="100"),
        DeclareLaunchArgument("exposure_profile", default_value="manual_indoor"),
        DeclareLaunchArgument("exposure", default_value="10"),
        DeclareLaunchArgument("gain", default_value="0"),
        DeclareLaunchArgument("trigger_mode", default_value="true"),
        # camera_node decodes MJPEG to mono8 in-process; raw_scale_denom is
        # libjpeg's DCT scaling ratio (1/2/4/8), so 2 -> 640x400, 4 -> 320x200.
        # The VIO calibration must match whatever this produces.
        DeclareLaunchArgument("publish_raw", default_value="true"),
        DeclareLaunchArgument("raw_scale_denom", default_value="2"),
    ]

    return LaunchDescription(args + [
        Node(
            package="v4l2_trigger_camera",
            executable="camera_node",
            name="camera",
            namespace="camera",
            output="screen",
            # The camera drops off the USB bus when the UPS switches over, and
            # comes back on a different /dev/video* index. Respawn so the node
            # re-opens it rather than sitting there publishing nothing.
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                "device": ParameterValue(LaunchConfiguration("device"), value_type=str),
                "width": ParameterValue(LaunchConfiguration("width"), value_type=int),
                "height": ParameterValue(LaunchConfiguration("height"), value_type=int),
                "fps": ParameterValue(LaunchConfiguration("fps"), value_type=int),
                "buffer_count": 4,
                "frame_id": "camera",
                "trigger_mode": ParameterValue(LaunchConfiguration("trigger_mode"), value_type=bool),
                "exposure_profile": ParameterValue(LaunchConfiguration("exposure_profile"), value_type=str),
                "exposure": ParameterValue(LaunchConfiguration("exposure"), value_type=int),
                "gain": ParameterValue(LaunchConfiguration("gain"), value_type=int),
                # Prefer the GPIO trigger timestamp: the V4L2 buffer timestamp
                # on this camera is quantised to a 4 ms grid.
                "use_trigger_stamp": True,
                "trigger_max_delay_ms": 50.0,
                "stats_every_frames": 300,
            
                "publish_raw": ParameterValue(
                    LaunchConfiguration("publish_raw"), value_type=bool),
                "raw_scale_denom": ParameterValue(
                    LaunchConfiguration("raw_scale_denom"), value_type=int),}],
        ),
    ])
