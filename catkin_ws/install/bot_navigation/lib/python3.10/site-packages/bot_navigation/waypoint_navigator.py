from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped

rclpy.init()
navigator = BasicNavigator()

# 等待 Nav2 启动
navigator.waitUntilNav2Active()

# 设置初始点
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
navigator.setInitialPose(initial_pose)

# 定义多个目标点
goal_poses = []

def create_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose

goal_poses.append(create_pose(1.0, 0.0))
goal_poses.append(create_pose(2.0, 1.0))
goal_poses.append(create_pose(0.0, 2.0))

# 发送 waypoint
navigator.followWaypoints(goal_poses)

while not navigator.isTaskComplete():
    pass

print("All waypoints reached!")