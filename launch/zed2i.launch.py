from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('zed_open_capture_ros')
    default_config = os.path.join(
        package_share,
        'config',
        'zed2i.yaml',
    )
    rviz_config = os.path.join(package_share, 'rviz', 'zed_stereo.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='zed'),
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('resolution', default_value='HD720'),
        DeclareLaunchArgument('frame_rate', default_value='30'),
        DeclareLaunchArgument('recording_mode', default_value='none'),
        DeclareLaunchArgument('recording_queue_depth', default_value='4'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz2 with the ZED stereo image layout'),
        Node(
            package='zed_open_capture_ros',
            executable='zed_open_capture_node',
            name='zed_open_capture',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'resolution': ParameterValue(
                        LaunchConfiguration('resolution'), value_type=str),
                    'frame_rate': ParameterValue(
                        LaunchConfiguration('frame_rate'), value_type=int),
                    'recording_mode': ParameterValue(
                        LaunchConfiguration('recording_mode'), value_type=str),
                    'recording_queue_depth': ParameterValue(
                        LaunchConfiguration('recording_queue_depth'), value_type=int),
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
