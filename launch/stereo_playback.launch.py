import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _playback(context):
    bag = os.path.abspath(os.path.expanduser(LaunchConfiguration('bag').perform(context)))
    mode = LaunchConfiguration('recording_mode').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    rate = LaunchConfiguration('rate').perform(context)
    warmup_seconds = float(LaunchConfiguration('warmup_seconds').perform(context))
    recorded_override = LaunchConfiguration('recorded_stereo_topic').perform(context)
    prefix = f'/{namespace}' if namespace else ''
    if not os.path.exists(bag):
        raise RuntimeError(f'bag does not exist: {bag}')
    if mode == 'slam_mono':
        suffix = 'mono'
    elif mode == 'stereo_color':
        suffix = 'color'
    else:
        raise RuntimeError('recording_mode must be slam_mono or stereo_color')

    combined = f'{prefix}/stereo/image_rect_{suffix}'
    recorded = recorded_override or f'{prefix}/stereo/recording/image_rect_{suffix}'
    left = f'{prefix}/left/image_rect_{suffix}'
    right = f'{prefix}/right/image_rect_{suffix}'
    topics = [
        recorded,
        f'{prefix}/left/camera_info',
        f'{prefix}/right/camera_info',
        f'{prefix}/imu/data_sync',
        '/tf_static',
    ]
    splitter = Node(
        package='zed_open_capture_ros',
        executable='stereo_splitter_node',
        name='zed_stereo_splitter',
        output='screen',
        parameters=[{
            'input_topic': combined,
            'left_output_topic': left,
            'right_output_topic': right,
        }],
    )
    # Publish calibration first, then give consumers such as GF-GG time to
    # load their vocabulary before the full image sequence begins.
    info_topics = [f'{prefix}/left/camera_info', f'{prefix}/right/camera_info']
    warmup = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag,
            '--playback-duration', '0.2',
            '--disable-keyboard-controls',
            '--topics', *info_topics,
        ],
        output='screen',
    )
    player_command = [
        'ros2', 'bag', 'play', bag,
        '--rate', rate,
        '--delay', '1.0',
        '--disable-keyboard-controls',
    ]
    if recorded != combined:
        player_command.extend(['--remap', f'{recorded}:={combined}'])
    player_command.extend(['--topics', *topics])
    player = ExecuteProcess(
        cmd=player_command,
        output='screen',
    )
    start_player = RegisterEventHandler(OnProcessExit(
        target_action=warmup,
        on_exit=[TimerAction(period=warmup_seconds, actions=[player])],
    ))
    return [splitter, start_player, warmup]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        DeclareLaunchArgument('recording_mode', default_value='slam_mono'),
        DeclareLaunchArgument('namespace', default_value='zed'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('warmup_seconds', default_value='4.0'),
        DeclareLaunchArgument('recorded_stereo_topic', default_value=''),
        OpaqueFunction(function=_playback),
    ])
