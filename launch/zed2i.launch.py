from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('zed_open_capture_ros'),
        'config',
        'zed2i.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='zed'),
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('resolution', default_value='HD720'),
        DeclareLaunchArgument('frame_rate', default_value='30'),
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
                },
            ],
        ),
    ])
