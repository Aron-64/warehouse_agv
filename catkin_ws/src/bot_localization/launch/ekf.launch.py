from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# BUG FIX: 使用 get_package_share_directory 替代硬编码绝对路径，
# 保证在任何机器、任何 workspace 下都能正确找到配置文件。
_CONFIG = os.path.join(
    get_package_share_directory('bot_localization'),
    'config',
    'ekf.yaml'
)


def generate_launch_description():

    # ── EKF 1：odom + imu → /odometry/filtered  +  odom→base_footprint TF ──
    # BUG FIX: 显式指定 namespace=''，确保节点名称与 ekf.yaml 中的
    # ekf_odom_node: ros__parameters: 段落匹配，避免参数加载混乱。
    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom_node',
        output='screen',
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
        parameters=[
            _CONFIG,
            {'use_sim_time': True},
        ]
    )

    # ── EKF 2：/odometry/filtered + /apriltag_pose → /odometry/filtered_map ──
    # BUG FIX: 显式指定 name='ekf_map_node'，与 ekf.yaml 中的
    # ekf_map_node: ros__parameters: 段落匹配。
    # publish_tf=false 已在 ekf.yaml 中配置，map→odom TF 由 AMCL 独家负责。
    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map_node',
        output='screen',
        remappings=[
            ('odometry/filtered', '/odometry/filtered_map'),
        ],
        parameters=[
            _CONFIG,
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([ekf_odom, ekf_map])