from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_share = FindPackageShare("bot_description").find("bot_description")

    xacro_file = os.path.join(pkg_share, "urdf/robot/robot.urdf.xacro")

    rviz_config = os.path.join(pkg_share, "config/display_robot_model.rviz")

    robot_description = Command(["xacro ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])