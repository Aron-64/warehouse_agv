from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    bot_nav      = get_package_share_directory('bot_navigation')

    nav2_params = os.path.join(bot_nav, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(bot_nav, 'maps', 'map.yaml')

    # ── nav2 完整 bringup（含 AMCL + map_server + 导航全栈）───────────────────
    # AMCL 的 tf_broadcast=true 已在 nav2_params.yaml 中配置
    # ekf_map_node 的 publish_tf=false 已在 ekf.yaml 中配置
    # 两者不再冲突，可以安全使用 bringup_launch.py
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

    # ── apriltag_relocator：将 EKF map 位姿定期发给 AMCL 做粒子重置校正 ───────
    relocator_node = Node(
        package='bot_apriltag',
        executable='apriltag_relocator',
        name='apriltag_relocator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── waypoint navigator ──────────────────────────────────────────────────
    waypoint_node = Node(
        package='bot_navigation',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        nav2_launch,
        relocator_node,
        waypoint_node,
    ])