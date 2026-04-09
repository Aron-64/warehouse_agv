"""
apriltag_relocator.py  (修复版)
────────────────────────────────
核心修复：
  Bug 1 — 触发逻辑反转
    原版在 _ekf_map_cb (50Hz) 里轮询 tag 是否可见，导致只要 tag 在超时窗口内
    出现过，之后每一帧 EKF 消息都会尝试触发重定位，冷却时间根本挡不住高频干扰。
    修复：将重定位判断移到 _tag_visible_cb 里触发（tag 驱动，不是 EKF 驱动）。
    EKF 回调只负责保存最新位姿，不再触发任何重定位逻辑。

  Bug 3 — 冷却时间过短
    HIGH 可信度下系数 0.4 × 5s = 2s，导航过程中经过多 tag 货架时每 2s 就
    重置一次粒子群，DWB 控制器反复收敛失败。
    修复：默认 cooldown 提高到 15s，HIGH 等级系数改为 2.0（最短 30s），
    LOW 等级更保守改为 3.0，同时新增 nav_active_cooldown_scale 参数，
    在导航任务进行中进一步延长冷却（AMCL 已初始化后再乘以 2.0 倍）。

  原有修复保留：
    - /initialpose 协方差下界（防止 AMCL 粒子过度收敛）
    - 可信度等级自动缩放阈值
    - 首次初始化逻辑（amcl_pose 为 None 时直接发送）
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

_RESET  = '\033[0m'
_BOLD   = '\033[1m'
_GREEN  = '\033[92m'
_YELLOW = '\033[93m'
_CYAN   = '\033[96m'
_GREY   = '\033[90m'
_BLUE   = '\033[94m'

_TRUST_HIGH = 'HIGH'
_TRUST_MED  = 'MED'
_TRUST_LOW  = 'LOW'

# Bug 3 修复：所有等级的冷却系数都大幅提高
# HIGH: 2.0 × 15s = 30s  （原来 0.4 × 5s = 2s，严重偏短）
# MED : 1.0 × 15s = 15s
# LOW : 3.0 × 15s = 45s  （远处单 tag，更保守）
_TRUST_SCALE = {
    _TRUST_HIGH: {'pos': 0.5, 'yaw': 0.5, 'cool': 2.0},
    _TRUST_MED:  {'pos': 1.0, 'yaw': 1.0, 'cool': 1.0},
    _TRUST_LOW:  {'pos': 2.0, 'yaw': 2.0, 'cool': 3.0},
}


class AprilTagRelocator(Node):

    def __init__(self):
        super().__init__('apriltag_relocator')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.Parameter.Type.BOOL, True)
        ])

        # ── 可调参数 ──────────────────────────────────────────
        self.declare_parameter('position_threshold',       0.3)
        self.declare_parameter('yaw_threshold',            0.15)
        self.declare_parameter('cooldown_sec',             15.0)   # Bug 3 修复: 5 → 15
        self.declare_parameter('tag_timeout_sec',          2.0)
        self.declare_parameter('cov_high_threshold',       0.02)
        self.declare_parameter('cov_low_threshold',        0.08)
        self.declare_parameter('initialpose_min_cov_xy',   0.10)
        self.declare_parameter('initialpose_min_cov_yaw',  0.05)
        # Bug 3 追加：AMCL 已初始化（导航进行中）时额外冷却倍率
        self.declare_parameter('nav_active_cooldown_scale', 2.0)

        self._pos_thr           = self.get_parameter('position_threshold').value
        self._yaw_thr           = self.get_parameter('yaw_threshold').value
        self._cooldown          = self.get_parameter('cooldown_sec').value
        self._tag_timeout       = self.get_parameter('tag_timeout_sec').value
        self._cov_high_thr      = self.get_parameter('cov_high_threshold').value
        self._cov_low_thr       = self.get_parameter('cov_low_threshold').value
        self._min_cov_xy        = self.get_parameter('initialpose_min_cov_xy').value
        self._min_cov_yaw       = self.get_parameter('initialpose_min_cov_yaw').value
        self._nav_cool_scale    = self.get_parameter('nav_active_cooldown_scale').value

        # ── 状态 ──────────────────────────────────────────────
        # Bug 1 修复：EKF 回调只写这个缓存，不再触发任何判断
        self._last_ekf_pose:   PoseWithCovarianceStamped | None = None
        self._amcl_pose:       PoseWithCovarianceStamped | None = None
        self._last_tag_time:   float = 0.0
        self._last_reloc_time: float = 0.0
        self._last_tag_cov_x:  float = 9999.0

        # ── 订阅 ──────────────────────────────────────────────
        # Bug 1 修复：EKF 回调只保存位姿，不触发重定位
        self.create_subscription(
            Odometry,
            '/odometry/filtered_map',
            self._ekf_map_cb,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_cb,
            10
        )

        # Bug 1 修复：重定位在这里触发（tag 驱动，而非 EKF 驱动）
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
            f'╔══════════════════════════════════════════════════╗\n'
            f'║   AprilTag Relocator Started (Fixed)             ║\n'
            f'║   pos_thr={self._pos_thr:.2f}m  '
            f'yaw_thr={math.degrees(self._yaw_thr):.1f}°              ║\n'
            f'║   cooldown={self._cooldown:.1f}s  '
            f'tag_timeout={self._tag_timeout:.1f}s           ║\n'
            f'║   cov→HIGH<{self._cov_high_thr}  LOW>{self._cov_low_thr}          ║\n'
            f'║   initialpose cov floor: '
            f'xy={self._min_cov_xy}  yaw={self._min_cov_yaw}     ║\n'
            f'║   [FIXED] Trigger: tag-driven, not EKF-driven    ║\n'
            f'╚══════════════════════════════════════════════════╝'
            f'{_RESET}'
        )

    # ──────────────────────────────────────────────────────── #
    #  EKF 回调：只保存位姿，不触发重定位                      #
    #  Bug 1 修复：原版在这里判断并触发，改为纯数据缓存         #
    # ──────────────────────────────────────────────────────── #

    def _ekf_map_cb(self, msg: Odometry):
        ekf_pose = PoseWithCovarianceStamped()
        ekf_pose.header          = msg.header
        ekf_pose.pose.pose       = msg.pose.pose
        ekf_pose.pose.covariance = list(msg.pose.covariance)
        self._last_ekf_pose = ekf_pose

    # ──────────────────────────────────────────────────────── #
    #  Tag 回调：更新时间戳 + 协方差 + 触发重定位判断          #
    #  Bug 1 修复：重定位逻辑从 _ekf_map_cb 移到这里           #
    # ──────────────────────────────────────────────────────── #

    def _tag_visible_cb(self, msg: PoseWithCovarianceStamped):
        self._last_tag_time  = self._now()
        self._last_tag_cov_x = msg.pose.covariance[0]

        # EKF 位姿还未就绪，等下一帧
        if self._last_ekf_pose is None:
            return

        self._try_relocate()

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        self._amcl_pose = msg

    # ──────────────────────────────────────────────────────── #
    #  重定位核心逻辑（从 _ekf_map_cb 提取重构）               #
    # ──────────────────────────────────────────────────────── #

    def _try_relocate(self):
        now = self._now()

        # ── 冗余保护：tag 必须在超时窗口内 ───────────────────
        if now - self._last_tag_time > self._tag_timeout:
            return

        # ── 根据可信度等级缩放阈值 ────────────────────────────
        trust    = self._trust_level()
        scale    = _TRUST_SCALE[trust]
        pos_thr  = self._pos_thr  * scale['pos']
        yaw_thr  = self._yaw_thr  * scale['yaw']
        cooldown = self._cooldown * scale['cool']

        # Bug 3 追加：AMCL 已初始化说明导航任务可能正在进行，
        # 再乘以额外倍率防止频繁重置粒子群
        if self._amcl_pose is not None:
            cooldown *= self._nav_cool_scale

        # ── 冷却时间检查 ──────────────────────────────────────
        if now - self._last_reloc_time < cooldown:
            self.get_logger().debug(
                f'{_GREY}[Relocator][{trust}] In cooldown '
                f'({now - self._last_reloc_time:.1f}s / {cooldown:.1f}s), skip.{_RESET}'
            )
            return

        # ── 构造 /initialpose（加协方差下界）────────────────────
        ekf_pose = self._last_ekf_pose
        cov = list(ekf_pose.pose.covariance)
        cov[0]  = max(cov[0],  self._min_cov_xy)
        cov[7]  = max(cov[7],  self._min_cov_xy)
        cov[35] = max(cov[35], self._min_cov_yaw)

        # 创建新消息以免修改缓存
        out = PoseWithCovarianceStamped()
        out.header          = ekf_pose.header
        out.pose.pose       = ekf_pose.pose.pose
        out.pose.covariance = cov

        # ── 与 AMCL 当前位姿比较，偏差足够大才触发 ───────────
        if self._amcl_pose is not None:
            pos_err, yaw_err = self._pose_diff(out, self._amcl_pose)

            if pos_err < pos_thr and yaw_err < yaw_thr:
                self.get_logger().debug(
                    f'{_GREY}[Relocator][{trust}] Drift small '
                    f'(Δpos={pos_err:.3f}m/{pos_thr:.3f}  '
                    f'Δyaw={math.degrees(yaw_err):.1f}°/{math.degrees(yaw_thr):.1f}°), '
                    f'skip.{_RESET}'
                )
                return

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
            # AMCL 尚未初始化，直接发送第一帧完成冷启动
            self.get_logger().info(
                f'\n{_GREEN}{_BOLD}'
                f'[Relocator][trust={trust}] AMCL not yet initialized, '
                f'sending first /initialpose from AprilTag EKF.'
                f'{_RESET}'
            )

        # ── 发布 /initialpose ──────────────────────────────────
        self._initialpose_pub.publish(out)
        self._last_reloc_time = now

        x   = out.pose.pose.position.x
        y   = out.pose.pose.position.y
        yaw = self._quat_to_yaw(out.pose.pose.orientation)
        self.get_logger().info(
            f'{_GREEN}'
            f'[Relocator] /initialpose published: '
            f'x={x:+.3f}  y={y:+.3f}  yaw={math.degrees(yaw):+.1f}°  '
            f'cov_xy={cov[0]:.3f}  cov_yaw={cov[35]:.3f}'
            f'{_RESET}'
        )

    # ──────────────────────────────────────────────────────── #

    def _trust_level(self) -> str:
        cov = self._last_tag_cov_x
        if cov < self._cov_high_thr:
            return _TRUST_HIGH
        elif cov < self._cov_low_thr:
            return _TRUST_MED
        else:
            return _TRUST_LOW

    def _pose_diff(self,
                   a: PoseWithCovarianceStamped,
                   b: PoseWithCovarianceStamped):
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
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _now(self) -> float:
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9


# ────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagRelocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()