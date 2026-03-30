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

    # ── nav2 bringup ────────────────────────────────────────────────────────
    # localization=false：不让 bringup 自己启动 AMCL 对应的 localization launch，
    # 我们在下面手动启动 amcl 节点，这样可以传入 tf_broadcast: false 的 params。
    #
    # 注意：nav2_bringup 的 bringup_launch.py 默认会同时启动 localization，
    # 如果你的 nav2_bringup 版本支持 'localization' 参数可以直接关掉；
    # 如果不支持，则用 navigation_launch.py 代替（只启动导航，不启动定位）。
    # ────────────────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # 只启动导航部分（controller/planner/bt_navigator 等），不包含 amcl/map_server
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':    'true',
            'params_file':     nav2_params,
            'use_composition': 'True',
        }.items()
    )

    # ── map_server：独立启动，提供静态地图 ──────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True},
        ]
    )

    # ── lifecycle_manager for map_server ────────────────────────────────────
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time':  True,
            'autostart':     True,
            'node_names':    ['map_server'],
        }]
    )

    # ── AMCL：独立启动，tf_broadcast=false 防止与 ekf_map_node 的 TF 冲突 ───
    # AMCL 仍然订阅 /scan 做粒子滤波，提升激光定位精度；
    # 但它不再发布 map→odom TF，该 TF 完全由 ekf_map_node 负责。
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': True},
        ]
    )

    # ── lifecycle_manager for amcl ──────────────────────────────────────────
    amcl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time':  True,
            'autostart':     True,
            'node_names':    ['amcl'],
        }]
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
        map_server,
        map_lifecycle,
        amcl_node,
        amcl_lifecycle,
        nav2_launch,
        waypoint_node,
    ])