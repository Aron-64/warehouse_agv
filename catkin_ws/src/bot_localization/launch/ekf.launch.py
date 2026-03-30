from launch import LaunchDescription
from launch_ros.actions import Node
import os

_CONFIG = '/home/agv/Desktop/catkin/catkin_ws/src/bot_localization/config/ekf.yaml'

def generate_launch_description():

    # ── EKF 1：odom + imu → /odometry/filtered  +  odom→base_footprint TF ──
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

    # ── EKF 2：/odometry/filtered + /apriltag_pose → map→odom TF ────────────
    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map_node',
        output='screen',
        remappings=[
            ('odometry/filtered', '/odometry/filtered_map'),  # 第二个EKF的输出单独命名，避免冲突
        ],
        parameters=[
            _CONFIG,
            {'use_sim_time': True},
        ]
    )

    return LaunchDescription([ekf_odom, ekf_map])