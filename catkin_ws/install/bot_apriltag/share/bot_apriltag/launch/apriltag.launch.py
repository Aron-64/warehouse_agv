from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    apriltag_launch_node = Node(
        package='bot_apriltag',
        executable='apriltag_node',
        output='screen'
    )

    return LaunchDescription([
        apriltag_launch_node,
    ])