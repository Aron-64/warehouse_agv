"""
apriltag_relocator.py
─────────────────────
订阅 ekf_map_node 输出的 /odometry/filtered_map，
在满足以下条件时将其转换为 /initialpose 发给 AMCL，触发粒子重新收敛。

  触发条件（同时满足）：
    1. AprilTag 可见且可信度足够（由 /apriltag_pose 的协方差大小决定）
    2. EKF map 位姿与 AMCL 当前位姿偏差超过阈值
    3. 距离上次触发超过冷却时间

  多 tag 处理：
    /apriltag_pose 由 apriltag_localizer 融合后发出，已是单条消息。
    本节点从该消息的协方差 cov[0]（x 方差）反推可信度等级：
      - HIGH  (cov_x < 0.02)：多个近距离 tag 融合 → 阈值收紧、冷却缩短，更积极校正
      - MED   (cov_x < 0.08)：单个中距离 tag     → 使用默认阈值
      - LOW   (cov_x >= 0.08)：tag 较远或单个远距  → 阈值放宽、冷却延长，保守校正

参数（均可通过 ROS2 参数覆盖）：
  position_threshold  : 触发重定位的位置偏差基准阈值（米），默认 0.3
  yaw_threshold       : 触发重定位的朝向偏差基准阈值（弧度），默认 0.15（≈8.6°）
  cooldown_sec        : 两次触发之间的基准最短间隔（秒），默认 5.0
  tag_timeout_sec     : 多久没收到 apriltag_pose 就认为 tag 不可见（秒），默认 1.0
  cov_high_threshold  : cov_x 低于此值视为 HIGH 可信度，默认 0.02
  cov_low_threshold   : cov_x 高于此值视为 LOW  可信度，默认 0.08
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# ── ANSI 颜色 ──────────────────────────────────────────────
_RESET  = '\033[0m'
_BOLD   = '\033[1m'
_GREEN  = '\033[92m'
_YELLOW = '\033[93m'
_CYAN   = '\033[96m'
_GREY   = '\033[90m'
_BLUE   = '\033[94m'

# ── 可信度等级 ─────────────────────────────────────────────
#   HIGH : 多个近距离 tag 融合，协方差很小
#   MED  : 单个中距离 tag
#   LOW  : 单个远距离 tag，协方差较大
_TRUST_HIGH = 'HIGH'
_TRUST_MED  = 'MED'
_TRUST_LOW  = 'LOW'

# 各等级对应的阈值缩放系数（相对于基准值）
# HIGH → 阈值收紧 0.5x、冷却缩短 0.4x，更积极校正
# MED  → 使用基准值
# LOW  → 阈值放宽 1.5x、冷却延长 1.5x，保守校正
_TRUST_SCALE = {
    _TRUST_HIGH: {'pos': 0.5, 'yaw': 0.5, 'cool': 0.4},
    _TRUST_MED:  {'pos': 1.0, 'yaw': 1.0, 'cool': 1.0},
    _TRUST_LOW:  {'pos': 1.5, 'yaw': 1.5, 'cool': 1.5},
}


class AprilTagRelocator(Node):

    def __init__(self):
        super().__init__('apriltag_relocator')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.Parameter.Type.BOOL, True)
        ])

        # ── 可调参数 ──────────────────────────────────────────
        self.declare_parameter('position_threshold', 0.3)
        self.declare_parameter('yaw_threshold',      0.15)
        self.declare_parameter('cooldown_sec',        5.0)
        self.declare_parameter('tag_timeout_sec',     1.0)
        self.declare_parameter('cov_high_threshold',  0.02)  # cov_x < 此值 → HIGH
        self.declare_parameter('cov_low_threshold',   0.08)  # cov_x > 此值 → LOW

        self._pos_thr      = self.get_parameter('position_threshold').value
        self._yaw_thr      = self.get_parameter('yaw_threshold').value
        self._cooldown     = self.get_parameter('cooldown_sec').value
        self._tag_timeout  = self.get_parameter('tag_timeout_sec').value
        self._cov_high_thr = self.get_parameter('cov_high_threshold').value
        self._cov_low_thr  = self.get_parameter('cov_low_threshold').value

        # ── 状态 ──────────────────────────────────────────────
        self._ekf_map_pose: PoseWithCovarianceStamped | None = None
        self._amcl_pose:    PoseWithCovarianceStamped | None = None
        self._last_tag_time:  float = 0.0
        self._last_reloc_time: float = 0.0
        # 最近一次 apriltag_pose 的 x 协方差（用于可信度判断）
        self._last_tag_cov_x: float = 9999.0

        # ── 订阅 ──────────────────────────────────────────────
        # ekf_map_node 输出（Odometry 类型，remapped 到 /ekf_map_pose）
        self.create_subscription(
            Odometry,
            '/odometry/filtered_map',
            self._ekf_map_cb,
            10
        )

        # AMCL 当前估计位姿（用于计算偏差）
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_cb,
            10
        )

        # AprilTag 原始位姿（仅用于判断 tag 是否可见，不处理内容）
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/apriltag_pose',
            self._tag_visible_cb,
            10
        )

        # ── 发布 ──────────────────────────────────────────────
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.get_logger().info(
            f'\n{_CYAN}{_BOLD}'
            f'╔══════════════════════════════════════════════╗\n'
            f'║   AprilTag Relocator Started                 ║\n'
            f'║   pos_thr={self._pos_thr:.2f}m  '
            f'yaw_thr={math.degrees(self._yaw_thr):.1f}°       ║\n'
            f'║   cooldown={self._cooldown:.1f}s  '
            f'tag_timeout={self._tag_timeout:.1f}s        ║\n'
            f'║   cov→HIGH<{self._cov_high_thr}  LOW>{self._cov_low_thr}       ║\n'
            f'╚══════════════════════════════════════════════╝'
            f'{_RESET}'
        )

    # ──────────────────────────────────────────────────────── #

    def _tag_visible_cb(self, msg: PoseWithCovarianceStamped):
        """
        记录最近一次收到 tag 的时间，同时保存协方差 cov[0]（x 方差）。
        apriltag_localizer 融合多 tag 后，tag 越多越近 → cov[0] 越小。
        """
        self._last_tag_time   = self._now()
        self._last_tag_cov_x  = msg.pose.covariance[0]  # 协方差矩阵 [0,0]

    def _trust_level(self) -> str:
        """根据最近一次 apriltag_pose 的 x 协方差判断可信度等级。"""
        cov = self._last_tag_cov_x
        if cov < self._cov_high_thr:
            return _TRUST_HIGH
        elif cov < self._cov_low_thr:
            return _TRUST_MED
        else:
            return _TRUST_LOW

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        self._amcl_pose = msg

    def _ekf_map_cb(self, msg: Odometry):
        """
        收到 ekf_map_node 的输出后，判断是否需要触发重定位。
        阈值和冷却时间根据 AprilTag 可信度等级动态缩放：
          HIGH（多近距tag）→ 更积极触发
          LOW （单远距tag）→ 更保守触发
        """
        now = self._now()

        # ── 条件 1：tag 必须在超时窗口内可见 ─────────────────────
        if now - self._last_tag_time > self._tag_timeout:
            self.get_logger().debug(
                f'{_GREY}[Relocator] Tag not visible, skip.{_RESET}'
            )
            return

        # ── 根据可信度等级缩放阈值 ────────────────────────────────
        trust   = self._trust_level()
        scale   = _TRUST_SCALE[trust]
        pos_thr  = self._pos_thr  * scale['pos']
        yaw_thr  = self._yaw_thr  * scale['yaw']
        cooldown = self._cooldown * scale['cool']

        # ── 条件 2：冷却时间（已按可信度缩放）───────────────────────
        if now - self._last_reloc_time < cooldown:
            return

        # ── 将 Odometry 转成 PoseWithCovarianceStamped ────────────
        ekf_pose = PoseWithCovarianceStamped()
        ekf_pose.header          = msg.header
        ekf_pose.pose.pose       = msg.pose.pose
        ekf_pose.pose.covariance = list(msg.pose.covariance)

        # ── 条件 3：与 AMCL 当前位姿偏差超过阈值才触发 ──────────────
        if self._amcl_pose is not None:
            pos_err, yaw_err = self._pose_diff(ekf_pose, self._amcl_pose)

            if pos_err < pos_thr and yaw_err < yaw_thr:
                self.get_logger().debug(
                    f'{_GREY}[Relocator][{trust}] Drift small '
                    f'(Δpos={pos_err:.3f}m/{pos_thr:.3f}  '
                    f'Δyaw={math.degrees(yaw_err):.1f}°/{math.degrees(yaw_thr):.1f}°), '
                    f'skip.{_RESET}'
                )
                return

            # 根据可信度选择日志颜色
            log_color = _GREEN if trust == _TRUST_HIGH else (
                _YELLOW if trust == _TRUST_MED else _CYAN)
            self.get_logger().info(
                f'\n{log_color}{_BOLD}'
                f'[Relocator][trust={trust}  cov_x={self._last_tag_cov_x:.4f}]\n'
                f'  Drift: Δpos={pos_err:.3f}m(thr={pos_thr:.3f})  '
                f'Δyaw={math.degrees(yaw_err):.1f}°(thr={math.degrees(yaw_thr):.1f}°)\n'
                f'  cooldown={cooldown:.1f}s  → Sending /initialpose to AMCL'
                f'{_RESET}'
            )
        else:
            self.get_logger().info(
                f'\n{_GREEN}{_BOLD}'
                f'[Relocator][trust={trust}] AMCL not yet initialized, '
                f'sending first /initialpose from AprilTag EKF.'
                f'{_RESET}'
            )

        # ── 发布 /initialpose ──────────────────────────────────────
        self._initialpose_pub.publish(ekf_pose)
        self._last_reloc_time = now

        x   = ekf_pose.pose.pose.position.x
        y   = ekf_pose.pose.pose.position.y
        yaw = self._quat_to_yaw(ekf_pose.pose.pose.orientation)
        self.get_logger().info(
            f'{_GREEN}'
            f'[Relocator] /initialpose published: '
            f'x={x:+.3f}  y={y:+.3f}  yaw={math.degrees(yaw):+.1f}°'
            f'{_RESET}'
        )

    # ──────────────────────────────────────────────────────── #

    def _pose_diff(self,
                   a: PoseWithCovarianceStamped,
                   b: PoseWithCovarianceStamped):
        """返回两个 PoseWithCovarianceStamped 之间的位置误差(m)和朝向误差(rad)。"""
        dx = a.pose.pose.position.x - b.pose.pose.position.x
        dy = a.pose.pose.position.y - b.pose.pose.position.y
        pos_err = math.sqrt(dx * dx + dy * dy)

        yaw_a = self._quat_to_yaw(a.pose.pose.orientation)
        yaw_b = self._quat_to_yaw(b.pose.pose.orientation)
        yaw_err = abs(math.atan2(
            math.sin(yaw_a - yaw_b),
            math.cos(yaw_a - yaw_b)
        ))
        return pos_err, yaw_err

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """从 geometry_msgs/Quaternion 提取 yaw。"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _now(self) -> float:
        """返回当前 ROS 时间（秒，float）。"""
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9


# ────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagRelocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()