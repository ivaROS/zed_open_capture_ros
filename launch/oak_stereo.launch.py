import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile


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

    # The DepthAI driver node alone. Its calibration TF (oak -> cameras, IMU) comes
    # from the device; the rig description (routemapper_tf.launch.py) publishes
    # base_link -> oak and /robot_description, so the depthai launch file (which also
    # starts a robot_state_publisher for the camera model) is not used.
    # Old:
    # actions = [
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(depthai_share, 'launch', 'driver.launch.py')),
    #         launch_arguments={
    #             'camera_model': 'OAK-D-PRO',
    #             'params_file': params_file,
    #             'use_rviz': 'false',
    #         }.items(),
    #     )
    # ]
    tf_params = {
        'driver': {
            'i_publish_tf_from_calibration': True,
            'i_tf_tf_prefix': 'oak',
            'i_tf_camera_model': '',
            'i_tf_base_frame': 'oak',
            'i_tf_parent_frame': 'base_link',
            'i_tf_cam_pos_x': '0.0',
            'i_tf_cam_pos_y': '0.0',
            'i_tf_cam_pos_z': '0.0',
            'i_tf_cam_roll': '0.0',
            'i_tf_cam_pitch': '0.0',
            'i_tf_cam_yaw': '0.0',
            'i_tf_imu_from_descr': 'false',
        }
    }
    actions = [
        ComposableNodeContainer(
            name='oak_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver_v3',
                    plugin='depthai_ros_driver::Driver',
                    name='oak',
                    namespace='',
                    parameters=[ParameterFile(params_file, allow_substs=True), tf_params],
                )
            ],
            arguments=['--ros-args', '--log-level', 'info'],
            output='both',
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
