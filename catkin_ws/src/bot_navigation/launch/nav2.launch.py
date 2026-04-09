"""
nav2_launch.py (含 rosbag 自动记录)

启动流程与原来完全相同，新增：
  - 自动启动 rosbag record，bag 保存到 ~/experiment_bags/<timestamp>/
  - 记录定位对比所需的全部 topic
  - 通过 launch argument 'record' 控制是否开启记录（默认开启）

用法：
  # 正常启动（自动记录）
  ros2 launch bot_navigation nav2_launch.py

  # 不记录 bag（调试用）
  ros2 launch bot_navigation nav2_launch.py record:=false
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    bot_nav      = get_package_share_directory('bot_navigation')

    nav2_params = os.path.join(bot_nav, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(bot_nav, 'maps', 'map.yaml')

    # ── 时间戳（启动时固定，避免文件名随重启变化）────────────────────
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir   = os.path.expanduser(f'/home/agv/Desktop/catkin/catkin_ws/src/experiment_bags/group_C_{timestamp}')

    # ── 记录的 topic ──────────────────────────────────────────────────
    # 三组实验共用同一份 topic 列表；分析时按需取用，不存在的 topic 自动跳过
    record_topics = [
        '/odom',                     # 原始轮速里程计        → 组A 基准
        '/imu',                      # IMU 原始数据
        '/odometry/filtered',        # EKF odom 融合结果    → 组A 定位
        '/odometry/filtered_map',    # EKF map 融合结果     → 组B/C 参考
        '/amcl_pose',                # AMCL 位姿估计        → 组B/C 定位
        '/apriltag_pose',            # AprilTag 定位输出    → 组C 定位
        '/plan',                     # Nav2 全局规划路径（作为参考基准）
        '/local_plan',               # DWB 局部规划轨迹
        '/tf',                       # TF 动态变换
        '/tf_static',                # TF 静态变换
        '/scan',                     # 激光雷达
        '/tag_detections',           # 原始 tag 检测        → 组C
        '/initialpose',              # relocator 重定位指令  → 组C
    ]

    # ── rosbag record 进程 ────────────────────────────────────────────
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', bag_dir,
            '--storage', 'sqlite3',
        ] + record_topics,
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
    )

    # ── 关键修复：在 bringup 启动前把地图路径写入全局参数 ────────────
    set_map_yaml = SetParameter(
        name='yaml_filename',
        value=map_file
    )

    # ── Nav2 完整 bringup ─────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time':    'true',
            'params_file':     nav2_params,
            'map':             map_file,
            'use_composition': 'True',
        }.items()
    )

    # ── AprilTag 重定位节点（延迟 3s 等 Nav2 容器就绪）───────────────
    relocator_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='bot_apriltag',
            executable='apriltag_relocator',
            name='apriltag_relocator',
            output='screen',
            parameters=[{
                'use_sim_time':              True,
                'cooldown_sec':              15.0,
                'nav_active_cooldown_scale': 2.0,
                'position_threshold':        0.3,
                'yaw_threshold':             0.15,
                'initialpose_min_cov_xy':    0.10,
                'initialpose_min_cov_yaw':   0.05,
            }]
        )]
    )

    # ── Waypoint 导航节点（延迟 8s 等 Nav2 完全激活）─────────────────
    waypoint_node = TimerAction(
        period=8.0,
        actions=[Node(
            package='bot_navigation',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'record',
            default_value='true',
            description='是否启动 rosbag 记录（true/false）',
            choices=['true', 'false'],
        ),

        bag_record,        # 最先启动，确保不丢开头的 TF/plan 数据
        set_map_yaml,
        nav2_launch,
        relocator_node,
        waypoint_node,
    ])