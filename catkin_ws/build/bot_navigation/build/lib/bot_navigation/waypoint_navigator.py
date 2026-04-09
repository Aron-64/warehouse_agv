from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


def make_pose(navigator, x, y, yaw_deg=0.0):
    """
    创建一个带时间戳的 PoseStamped。
    yaw_deg: 目标朝向（度），0=朝正东，90=朝正北，-90=朝正南，180=朝正西
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    # 将 yaw 角转换为四元数（只绕 Z 轴旋转）
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 等待 Nav2 全栈就绪
    # 注意：不调用 setInitialPose，初始位姿完全由 apriltag_relocator 提供
    navigator.waitUntilNav2Active()

    navigator.get_logger().info('Nav2 is ready for use!')
    navigator.get_logger().info('Following 5 goals....')

    # ── 定义巡航路径 ──────────────────────────────────────────────────────
    # 坐标来自 tag_map.yaml 附近的路径点

    goal_poses = [
        make_pose(navigator,  2.0, -1.0, yaw_deg=-25.0),
        make_pose(navigator,  2.0, -5.0, yaw_deg=-90.0),
        make_pose(navigator, -0.5, -5.0, yaw_deg=180.0),
        make_pose(navigator, -0.5, -1.0, yaw_deg= 90.0),
        make_pose(navigator,  2.0, -1.0, yaw_deg=25.0),

        # make_pose(navigator,  2.0, -1.0, yaw_deg=-90.0),
        # make_pose(navigator,  2.0, -5.0, yaw_deg=180.0),
        # make_pose(navigator, -0.5, -5.0, yaw_deg= 90.0),
        # make_pose(navigator, -0.5, -1.0, yaw_deg=  0.0),
        # make_pose(navigator,  2.0, -1.0, yaw_deg=-90.0),
    ]

    # 发送 waypoint 列表，stop_on_failure=false 已在 nav2_params.yaml 中配置
    navigator.followWaypoints(goal_poses)

    # 等待任务完成
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info(
                f'Executing waypoint '
                f'{feedback.current_waypoint + 1}/{len(goal_poses)}'
            )

    result = navigator.getResult()
    navigator.get_logger().info(f'Navigation result: {result}')
    navigator.get_logger().info('All waypoints reached!')

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()