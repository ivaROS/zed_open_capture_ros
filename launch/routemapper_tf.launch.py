#!/usr/bin/env python3
# Routemapper backpack: publish the rig description (config/routemapper.urdf) on
# /robot_description and its fixed joints on /tf_static: base_link -> base_laser,
# zed_left_camera_frame, oak and theta_link. The sensor drivers publish the frames
# below their roots. Values and their sources are in the URDF.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('zed_open_capture_ros'), 'config', 'routemapper.urdf')
    with open(urdf) as f:
        description = f.read()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='rig_state_publisher', output='log',
             parameters=[{'robot_description': description}]),
    ])
