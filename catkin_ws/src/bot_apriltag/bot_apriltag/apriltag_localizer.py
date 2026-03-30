import os

import rclpy
from rclpy.node import Node

import yaml
import numpy as np
import tf_transformations

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener


class AprilTagLocalizer(Node):

    def __init__(self):
        super().__init__('apriltag_localizer')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])
        # 读取tag地图
        pkg_path = get_package_share_directory('bot_apriltag')
        yaml_path = os.path.join(pkg_path, 'config', 'tag_map.yaml')

        with open(yaml_path, 'r') as f:
            self.tag_map = yaml.safe_load(f)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅检测
        self.sub = self.create_subscription(
            TransformStamped,
            '/tag_detections',
            self.callback,
            10
        )

        # 输出定位
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag_pose',
            10
        )

        self.get_logger().info("AprilTag Localizer Started")

    def callback(self, msg):

        tag_id = msg.child_frame_id

        if tag_id not in self.tag_map:
            return

        # =========================
        # 1️⃣ map → tag
        # =========================
        tag = self.tag_map[tag_id]

        T_map_tag = tf_transformations.compose_matrix(
            translate=[tag['x'], tag['y'], tag['z']],
            angles=[tag['roll'], tag['pitch'], tag['yaw']]
        )

        # =========================
        # 2️⃣ tag → camera
        # =========================
        t = msg.transform.translation
        r = msg.transform.rotation

        T_tag_camera = tf_transformations.quaternion_matrix(
            [r.x, r.y, r.z, r.w]
        )
        T_tag_camera[0][3] = t.x
        T_tag_camera[1][3] = t.y
        T_tag_camera[2][3] = t.z

        # 🚨 注意：PnP算的是 camera→tag，需要反一下
        T_camera_tag = np.linalg.inv(T_tag_camera)

        # =========================
        # 3️⃣ camera → base_link
        # =========================
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint',
                msg.header.frame_id,  # camera_link
                rclpy.time.Time()
            )
        except:
            self.get_logger().warn("TF lookup failed")
            return

        t = tf.transform.translation
        r = tf.transform.rotation

        T_camera_base = tf_transformations.quaternion_matrix(
            [r.x, r.y, r.z, r.w]
        )
        T_camera_base[0][3] = t.x
        T_camera_base[1][3] = t.y
        T_camera_base[2][3] = t.z

        # =========================
        # 🔥 核心计算
        # =========================
        T_map_base = T_map_tag @ T_camera_tag @ T_camera_base

        # =========================
        # 发布 Pose
        # =========================
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "map"

        pose.pose.pose.position.x = T_map_base[0][3]
        pose.pose.pose.position.y = T_map_base[1][3]

        # quat = tf_transformations.quaternion_from_matrix(T_map_base)

        # pose.pose.pose.orientation.x = quat[0]
        # pose.pose.pose.orientation.y = quat[1]
        # pose.pose.pose.orientation.z = quat[2]
        # pose.pose.pose.orientation.w = quat[3]

        # 提取 yaw
        yaw = tf_transformations.euler_from_matrix(T_map_base)[2]
        # 转回 quaternion（仅yaw）
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

        pose.pose.pose.orientation.x = quat[0]
        pose.pose.pose.orientation.y = quat[1]
        pose.pose.pose.orientation.z = quat[2]
        pose.pose.pose.orientation.w = quat[3]

        # # 协方差
        # pose.pose.covariance[0] = 0.05
        # pose.pose.covariance[7] = 0.05
        # pose.pose.covariance[35] = 0.1
        cov = [0.0] * 36
        # x, y
        cov[0] = 0.05
        cov[7] = 0.05
        # z（不用但必须给大值）
        cov[14] = 1.0
        # roll pitch
        cov[21] = 1.0
        cov[28] = 1.0
        # yaw
        cov[35] = 0.1

        pose.pose.covariance = cov

        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()