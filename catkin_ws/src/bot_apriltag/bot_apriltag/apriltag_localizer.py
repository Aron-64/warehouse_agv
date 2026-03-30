import os

import rclpy
from rclpy.node import Node

import yaml
import numpy as np
import tf_transformations

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

# ── ANSI 颜色常量 ──────────────────────────────────────────────
_RESET  = '\033[0m'
_BOLD   = '\033[1m'
_GREEN  = '\033[92m'
_YELLOW = '\033[93m'
_CYAN   = '\033[96m'
_RED    = '\033[91m'
_BLUE   = '\033[94m'


class AprilTagLocalizer(Node):

    # ── 融合窗口时长（秒）：在此时间内收集到的所有tag一起融合 ──
    FUSION_WINDOW_SEC = 0.05   # 50 ms，可按帧率调整

    def __init__(self):
        super().__init__('apriltag_localizer')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        # ── 读取 tag 地图 ──────────────────────────────────────
        pkg_path  = get_package_share_directory('bot_apriltag')
        yaml_path = os.path.join(pkg_path, 'config', 'tag_map.yaml')
        with open(yaml_path, 'r') as f:
            self.tag_map = yaml.safe_load(f)

        # ── TF ────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 多 tag 缓冲区 ──────────────────────────────────────
        # 每条记录：{'x', 'y', 'yaw', 'weight', 'dist', 'tag_id', 'stamp'}
        self._pending: list[dict] = []
        self._window_start = None   # 当前窗口的起始时间（float 秒）

        # ── 订阅检测 ───────────────────────────────────────────
        self.sub = self.create_subscription(
            TransformStamped,
            '/tag_detections',
            self.callback,
            10
        )

        # ── 输出定位 ───────────────────────────────────────────
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag_pose',
            10
        )

        self.get_logger().info(
            f'\n{_BLUE}{_BOLD}'
            f'╔══════════════════════════════════════╗\n'
            f'║   AprilTag Localizer Started         ║\n'
            f'║   fusion window = {self.FUSION_WINDOW_SEC*1000:.0f} ms            ║\n'
            f'╚══════════════════════════════════════╝'
            f'{_RESET}'
        )

    # ──────────────────────────────────────────────────────────────── #

    def callback(self, msg: TransformStamped):
        """
        每收到一个 tag 的检测消息就调用一次。
        先计算该 tag 对应的机器人 pose，缓存；
        当窗口超时后触发融合并发布。
        """

        tag_id = msg.child_frame_id          # e.g. "tag_3"

        if tag_id not in self.tag_map:
            return

        # ── 1. 计算当前 tag 对应的机器人 map pose ─────────────
        result = self._compute_pose_from_tag(msg, tag_id)
        if result is None:
            return

        x, y, yaw, dist = result

        # 权重 = 1/dist²（距离越近越可信），最小距离截断防除零
        dist   = max(dist, 0.01)
        weight = 1.0 / (dist ** 2)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ── 2. 缓冲窗口管理 ────────────────────────────────────
        if self._window_start is None:
            # 开启第一个窗口
            self._window_start = stamp_sec
            self._pending.clear()

        window_age = stamp_sec - self._window_start

        if window_age <= self.FUSION_WINDOW_SEC:
            # 窗口内：追加到缓冲
            self._pending.append({
                'x': x, 'y': y, 'yaw': yaw,
                'weight': weight,
                'dist': dist,
                'tag_id': tag_id,
                'stamp': msg.header.stamp,
            })
        else:
            # 窗口已超时：先用上一窗口缓冲发布，再开新窗口
            if self._pending:
                self._fuse_and_publish(self._pending)

            self._window_start = stamp_sec
            self._pending = [{
                'x': x, 'y': y, 'yaw': yaw,
                'weight': weight,
                'dist': dist,
                'tag_id': tag_id,
                'stamp': msg.header.stamp,
            }]

    # ──────────────────────────────────────────────────────────────── #

    def _compute_pose_from_tag(self, msg: TransformStamped, tag_id: str):
        """
        根据单个 tag 检测消息，计算机器人在 map 坐标系下的 (x, y, yaw, dist)。

        变换链（正确推导）：
          T_map_base = T_map_tag  ×  T_tag_camera  ×  T_camera_base

        其中：
          T_tag_camera  = inv(T_camera_tag)
              ↑ PnP 输出的是 tag 在 camera 坐标系下的位置，即 T_camera_tag
          T_camera_base = TF 查询 camera_link → base_footprint
              ↑ 注意查询方向：source=camera_link, target=base_footprint
        """

        # ── Step 1: map → tag（从 yaml 地图读取）─────────────────
        tag = self.tag_map[tag_id]
        T_map_tag = tf_transformations.compose_matrix(
            translate=[tag['x'], tag['y'], tag['z']],
            angles=[tag['roll'], tag['pitch'], tag['yaw']]
        )

        # ── Step 2: camera → tag（PnP 直接输出，即 T_camera_tag）──
        t = msg.transform.translation
        r = msg.transform.rotation
        T_camera_tag = tf_transformations.quaternion_matrix(
            [r.x, r.y, r.z, r.w]
        )
        T_camera_tag[0][3] = t.x
        T_camera_tag[1][3] = t.y
        T_camera_tag[2][3] = t.z

        # tag → camera = inv(T_camera_tag)
        T_tag_camera = np.linalg.inv(T_camera_tag)

        # ── Step 3: camera → base_link（TF 查询）──────────────────
        #   查询方向：source = camera_link，target = base_footprint
        #   得到的是 base_footprint 在 camera_link 坐标系下的位置，即 T_camera_base
        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id,   # camera_link（source）
                'base_footprint',      # base_footprint（target）
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        tc = tf.transform.translation
        rc = tf.transform.rotation
        T_camera_base = tf_transformations.quaternion_matrix(
            [rc.x, rc.y, rc.z, rc.w]
        )
        T_camera_base[0][3] = tc.x
        T_camera_base[1][3] = tc.y
        T_camera_base[2][3] = tc.z

        # ── Step 4: 核心链式计算 ───────────────────────────────────
        T_map_base = T_map_tag @ T_tag_camera @ T_camera_base

        x   = T_map_base[0][3]
        y   = T_map_base[1][3]
        yaw = tf_transformations.euler_from_matrix(T_map_base)[2]

        # 用 tag 在相机坐标系下的距离作为可信度依据
        dist = float(np.linalg.norm([t.x, t.y, t.z]))

        return x, y, yaw, dist

    # ──────────────────────────────────────────────────────────────── #

    def _fuse_and_publish(self, records: list[dict]):
        """
        对缓冲区内所有 tag 的定位结果做距离加权融合，发布一次 PoseWithCovarianceStamped。

        位置 (x, y)：加权平均
        朝向 (yaw) ：单位复数加权平均（避免角度翻转）
                     yaw_fused = atan2( Σ w·sin(yaw_i) , Σ w·cos(yaw_i) )
        协方差     ：根据参与融合的 tag 数量和平均距离自动缩放
        """

        weights = np.array([r['weight'] for r in records])
        W       = weights.sum()
        w_norm  = weights / W   # 归一化权重

        # ── 加权位置 ──────────────────────────────────────────────
        x_fused = float(np.sum([r['x'] * w for r, w in zip(records, w_norm)]))
        y_fused = float(np.sum([r['y'] * w for r, w in zip(records, w_norm)]))

        # ── 加权 yaw（复数平均，避免 ±π 翻转）────────────────────
        sin_sum   = float(np.sum([np.sin(r['yaw']) * w for r, w in zip(records, w_norm)]))
        cos_sum   = float(np.sum([np.cos(r['yaw']) * w for r, w in zip(records, w_norm)]))
        yaw_fused = float(np.arctan2(sin_sum, cos_sum))

        # ── 自动协方差缩放 ────────────────────────────────────────
        #   tag 越近（dist 小）、数量越多，方差越小
        n_tags   = len(records)
        avg_dist = float(np.mean([r['dist'] for r in records]))
        scale    = (avg_dist ** 2) / n_tags
        base_cov = 0.05

        cov_xy  = base_cov * scale
        cov_yaw = base_cov * 2.0 * scale

        cov = [0.0] * 36
        cov[0]  = max(cov_xy,  0.001)   # x
        cov[7]  = max(cov_xy,  0.001)   # y
        cov[14] = 1.0                    # z（2D 导航不用）
        cov[21] = 1.0                    # roll（不用）
        cov[28] = 1.0                    # pitch（不用）
        cov[35] = max(cov_yaw, 0.01)    # yaw

        # ── 构造消息 ───────────────────────────────────────────────
        pose = PoseWithCovarianceStamped()
        # 取缓冲区内最新的时间戳
        pose.header.stamp = max(
            records,
            key=lambda r: r['stamp'].sec + r['stamp'].nanosec * 1e-9
        )['stamp']
        pose.header.frame_id = 'map'

        pose.pose.pose.position.x = x_fused
        pose.pose.pose.position.y = y_fused
        pose.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_fused)
        pose.pose.pose.orientation.x = quat[0]
        pose.pose.pose.orientation.y = quat[1]
        pose.pose.pose.orientation.z = quat[2]
        pose.pose.pose.orientation.w = quat[3]

        pose.pose.covariance = cov

        self.pub.publish(pose)

        # ── 彩色日志 ──────────────────────────────────────────────
        tag_ids_str = ', '.join(r['tag_id'] for r in records)
        self.get_logger().info(
            f'\n{_GREEN}{_BOLD}'
            f'┌─────────────────────────────────────────────────┐\n'
            f'│  Fused {n_tags} tag(s): [{tag_ids_str}]\n'
            f'│  x={x_fused:+.3f}  y={y_fused:+.3f}  yaw={np.degrees(yaw_fused):+.1f}°\n'
            f'│  avg_dist={avg_dist:.2f}m  cov_xy={cov_xy:.4f}  cov_yaw={cov_yaw:.4f}\n'
            f'└─────────────────────────────────────────────────┘'
            f'{_RESET}'
        )


# ────────────────────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()