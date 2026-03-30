import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('bot_navigation'),
        'config', 'nav2_params.yaml'
    )

    # Humble 中各节点对应的包名
    nav2_node_configs = [
        ('nav2_bt_navigator',        'bt_navigator'),
        ('nav2_planner',             'planner_server'),
        ('nav2_controller',          'controller_server'),
        ('nav2_behaviors',           'recoveries_server'),   # Humble 改名
        ('nav2_waypoint_follower',   'waypoint_follower'),
    ]

    nodes = []
    for pkg, executable in nav2_node_configs:
        nodes.append(Node(
            package=pkg,
            executable=executable,
            name=executable,
            parameters=[nav2_params],
            output='screen',
        ))

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [cfg[1] for cfg in nav2_node_configs]
        }],
        output='screen',
    ))

    return LaunchDescription(nodes)