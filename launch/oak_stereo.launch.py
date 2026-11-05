import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package_share = get_package_share_directory('zed_open_capture_ros')
    depthai_share = get_package_share_directory('depthai_ros_driver_v3')
    frame_rate = LaunchConfiguration('frame_rate').perform(context)
    image_mode = LaunchConfiguration('image_mode').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    if frame_rate not in ('30', '60'):
        raise RuntimeError('frame_rate must be 30 or 60')
    if image_mode not in ('raw', 'rectified'):
        raise RuntimeError('image_mode must be raw or rectified')
    if not params_file:
        suffix = '_60hz' if frame_rate == '60' else ''
        params_file = os.path.join(
            package_share, 'config', f'oak_stereo_only{suffix}.yaml')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_share, 'launch', 'driver.launch.py')),
            launch_arguments={
                'camera_model': 'OAK-D-PRO',
                'params_file': params_file,
                'use_rviz': 'false',
            }.items(),
        )
    ]

    if image_mode == 'rectified':
        for side in ('left', 'right'):
            actions.append(
                Node(
                    package='image_proc',
                    executable='rectify_node',
                    namespace=f'oak/{side}',
                    name='rectify',
                    output='screen',
                    remappings=[
                        ('image', 'image_raw'),
                        ('camera_info', 'camera_info'),
                        ('image_rect', 'image_rect'),
                    ],
                )
            )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_rate', default_value='30'),
        DeclareLaunchArgument(
            'image_mode',
            default_value='raw',
            description='Publish raw only, or raw plus ROS-rectified stereo'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional DepthAI parameter file override'),
        OpaqueFunction(function=launch_setup),
    ])
