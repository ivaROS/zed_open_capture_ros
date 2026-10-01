import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    zed_launch = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py',
    )

    parameters = ';'.join([
        'general.grab_resolution:=HD720',
        'general.grab_frame_rate:=30',
        'general.publish_status:=true',
        'depth.depth_mode:=NONE',
        'depth.publish_depth_map:=false',
        'depth.publish_depth_image:=false',
        'depth.publish_depth_info:=false',
        'depth.publish_point_cloud:=false',
        'depth.publish_depth_confidence:=false',
        'depth.publish_disparity:=false',
        'video.enable_24bit_output:=true',
        'video.publish_rgb:=false',
        'video.publish_left_right:=true',
        'video.publish_raw:=false',
        'video.publish_gray:=false',
        'video.publish_stereo:=false',
        'sensors.publish_imu:=true',
        'sensors.publish_imu_raw:=true',
        'sensors.publish_cam_imu_transf:=true',
        'sensors.publish_mag:=true',
        'sensors.publish_baro:=true',
        'sensors.publish_temp:=true',
        'sensors.sensors_image_sync:=false',
        'pos_tracking.pos_tracking_enabled:=true',
        'pos_tracking.pos_tracking_mode:=GEN_3',
        'pos_tracking.compute_preference:=PREFER_CPU',
        'pos_tracking.imu_fusion:=true',
        'pos_tracking.area_memory:=false',
        'pos_tracking.publish_odom_pose:=true',
        'pos_tracking.publish_pose_cov:=true',
        'pos_tracking.publish_cam_path:=false',
        'pos_tracking.publish_3d_landmarks:=false',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('serial_number', default_value='39494516'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': LaunchConfiguration('camera_name'),
                'serial_number': LaunchConfiguration('serial_number'),
                'publish_tf': 'true',
                'publish_map_tf': 'true',
                'publish_imu_tf': 'true',
                'param_overrides': parameters,
            }.items(),
        ),
    ])
