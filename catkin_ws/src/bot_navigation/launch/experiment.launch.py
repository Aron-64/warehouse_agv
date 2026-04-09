"""
experiment_launch.py
────────────────────
三组对照实验的统一启动入口。

用法：
  # 组A：纯里程计（EKF odom only，map→odom TF 固定为 identity）
  ros2 launch bot_navigation experiment_launch.py group:=A

  # 组B：EKF + AMCL（激光雷达定位），无 AprilTag
  ros2 launch bot_navigation experiment_launch.py group:=B

  # 组C：完整方案（EKF + AMCL + AprilTag 重定位）
  ros2 launch bot_navigation experiment_launch.py group:=C

bag 文件保存在 ~/experiment_bags/group_<X>_<timestamp>/
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    group        = LaunchConfiguration('group').perform(context)
    nav2_bringup = get_package_share_directory('nav2_bringup')
    bot_nav      = get_package_share_directory('bot_navigation')

    map_file = os.path.join(bot_nav, 'maps', 'map.yaml')

    # ── 各组 nav2 params ───────────────────────────────────────────────
    params_map = {
        'A': os.path.join(bot_nav, 'config', 'nav2_params_group_a.yaml'),
        'B': os.path.join(bot_nav, 'config', 'nav2_params_group_b.yaml'),
        'C': os.path.join(bot_nav, 'config', 'nav2_params.yaml'),
    }
    nav2_params = params_map.get(group, params_map['C'])

    # ── rosbag 输出路径 ────────────────────────────────────────────────
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir   = os.path.expanduser(
        f'/home/agv/Desktop/catkin/catkin_ws/src/experiment_bags/group_{group}_{timestamp}'
    )

    # ── 记录 topic 列表（三组统一记录，分析时按需取用）────────────────
    record_topics = [
        '/odom',                    # 原始轮速里程计
        '/imu',                     # IMU 原始数据
        '/odometry/filtered',       # EKF odom 融合结果
        '/odometry/filtered_map',   # EKF map 融合结果（组B/C）
        '/amcl_pose',               # AMCL 位姿估计（组B/C）
        '/apriltag_pose',           # AprilTag 定位输出（组C）
        '/plan',                    # Nav2 全局规划路径
        '/local_plan',              # DWB 局部规划路径
        '/tf',
        '/tf_static',
        '/scan',
        '/tag_detections',          # 原始 tag 检测（组C）
        '/initialpose',             # relocator 重定位指令（组C）
    ]

   bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', bag_dir,
            '--storage', 'sqlite3',   # 改这里，去掉 mcap 相关两行
        ] + record_topics,
        output='screen',
        name=f'bag_record_group_{group}',
    )

    # ── 组A 专用：固定 map→odom identity TF ───────────────────────────
    # AMCL tf_broadcast=false 时 Nav2 找不到 map→odom，用 static 占位
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(['"', group, '" == "A"'])),
    )

    # ── Nav2 bringup（三组共用，params_file 不同）─────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time':    'true',
            'params_file':     nav2_params,
            'map':             map_file,
            'use_composition': 'True',
        }.items(),
    )

    # ── AprilTag 节点（仅组C）──────────────────────────────────────────
    is_group_c = IfCondition(PythonExpression(['"', group, '" == "C"']))

    apriltag_detector = Node(
        package='bot_apriltag', executable='apriltag_detector',
        name='apriltag_detector', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=is_group_c,
    )
    apriltag_localizer = Node(
        package='bot_apriltag', executable='apriltag_localizer',
        name='apriltag_localizer', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=is_group_c,
    )
    apriltag_relocator = Node(
        package='bot_apriltag', executable='apriltag_relocator',
        name='apriltag_relocator', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=is_group_c,
    )

    # ── Waypoint 导航（延迟 5s 等 Nav2 就绪）──────────────────────────
    waypoint_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='bot_navigation', executable='waypoint_navigator',
            name='waypoint_navigator', output='screen',
            parameters=[{'use_sim_time': True}],
        )]
    )

    return [
        static_map_odom,
        nav2_launch,
        apriltag_detector,
        apriltag_localizer,
        apriltag_relocator,
        bag_record,
        waypoint_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'group',
            default_value='C',
            description='A=odom only  B=EKF+AMCL  C=EKF+AMCL+AprilTag',
            choices=['A', 'B', 'C'],
        ),
        OpaqueFunction(function=launch_setup),
    ])