"""回放 launch：一键起 hand_controller + replay_targets，手重做录制的动作。

用法（先停掉遥操链，避免 retargeter 抢话题）：
  ros2 launch orcahand_teleop_ros2 replay.launch.py file:=~/fthand_ws/data/traj_xxx.csv
  # 循环回放加： loop:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('file', description='回放 CSV 文件路径（必填）'),
        DeclareLaunchArgument('loop', default_value='false', description='true=循环回放'),
        DeclareLaunchArgument('mock', default_value='false', description='false=驱动真手'),
        DeclareLaunchArgument(
            'model_path',
            default_value='~/FThand/FThand/FT_core/orca_core/models/orcahand_v1_left'),
        DeclareLaunchArgument('skip_joints', default_value='pinky_abd'),

        Node(package='orcahand_teleop_ros2', executable='hand_controller',
             name='hand_controller', output='screen',
             parameters=[{
                 'mock': LaunchConfiguration('mock'),
                 'model_path': LaunchConfiguration('model_path'),
                 'skip_joints': LaunchConfiguration('skip_joints'),
             }]),

        Node(package='orcahand_teleop_ros2', executable='replay_targets',
             name='replay_targets', output='screen',
             parameters=[{
                 'file': LaunchConfiguration('file'),
                 'loop': LaunchConfiguration('loop'),
             }]),
    ])
