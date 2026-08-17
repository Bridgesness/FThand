"""单机 launch：一键起 3 节点（手套源 + retargeter + hand_controller），全部参数可传。

用法：
  # 真手套 + 真手（默认，板子上自包含）
  ros2 launch orcahand_teleop_ros2 single_pc.launch.py

  # 遥操 + 同时录制（推荐数采流程，Ctrl+C 停时自动保存 CSV）：
  ros2 launch orcahand_teleop_ros2 single_pc.launch.py record:=true
  #   固定时长录制：record:=true record_duration:=10 （10 秒自动存）

  # 假手套 + mock（安全干跑，不碰硬件）
  ros2 launch orcahand_teleop_ros2 single_pc.launch.py use_fake_glove:=true mock:=true

  # 右手（记得换 model_path）
  ros2 launch orcahand_teleop_ros2 single_pc.launch.py hand:=right \
    model_path:=/home/ubuntu/FThand/FThand/FT_core/orca_core/models/orcahand_v1_right
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ============ 1. 声明运行时参数（启动时用 名字:=值 覆盖）============
        DeclareLaunchArgument(
            'use_fake_glove', default_value='false',
            description='true=假手套(fake_glove), false=真手套(glove_driver收UDP8000)'),
        DeclareLaunchArgument(
            'hand', default_value='left',
            description='哪只手: left / right'),
        DeclareLaunchArgument(
            'model_path',
            default_value='~/FThand/FThand/FT_core/orca_core/models/orcahand_v1_left',
            description='模型路径（右手换成 orcahand_v1_right；~ 会自动展开）'),
        DeclareLaunchArgument(
            'mock', default_value='false',
            description='true=hand_controller 不碰硬件(mock), false=驱动真手'),
        DeclareLaunchArgument(
            'skip_joints', default_value='pinky_abd',
            description='跳过的关节（逗号分隔），如 pinky_abd'),
        DeclareLaunchArgument(
            'motion_scale', default_value='0.8',
            description='运动缩放(0~1)'),
        DeclareLaunchArgument(
            'smoothing', default_value='0.3',
            description='平滑系数(越大越跟手, 越小越稳)'),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='true=同时启动录制(record_targets)，Ctrl+C 停时保存CSV'),
        DeclareLaunchArgument(
            'record_duration', default_value='0',
            description='录制时长秒，0=直到 Ctrl+C'),

        # ============ 2. 手套数据源：按 use_fake_glove 二选一 ============
        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_fake_glove')),
            actions=[
                Node(package='orcahand_teleop_ros2', executable='fake_glove',
                     name='fake_glove', output='screen'),
            ],
        ),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('use_fake_glove')),
            actions=[
                Node(package='orcahand_teleop_ros2', executable='glove_driver',
                     name='glove_driver', output='screen'),
            ],
        ),

        # ============ 3. retargeter：映射+平滑 ============
        Node(
            package='orcahand_teleop_ros2',
            executable='retargeter',
            name='retargeter',
            output='screen',
            parameters=[{
                'hand': LaunchConfiguration('hand'),
                'model_path': LaunchConfiguration('model_path'),
                'motion_scale': LaunchConfiguration('motion_scale'),
                'smoothing': LaunchConfiguration('smoothing'),
            }],
        ),

        # ============ 4. hand_controller：控手 ============
        Node(
            package='orcahand_teleop_ros2',
            executable='hand_controller',
            name='hand_controller',
            output='screen',
            parameters=[{
                'mock': LaunchConfiguration('mock'),
                'model_path': LaunchConfiguration('model_path'),
                'skip_joints': LaunchConfiguration('skip_joints'),
            }],
        ),

        # ============ 5. 可选录制（record:=true 时起 record_targets）============
        GroupAction(
            condition=IfCondition(LaunchConfiguration('record')),
            actions=[
                Node(package='orcahand_teleop_ros2', executable='record_targets',
                     name='record_targets', output='screen',
                     parameters=[{'duration': LaunchConfiguration('record_duration')}]),
            ],
        ),
    ])
