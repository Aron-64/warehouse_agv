import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped

from cv_bridge import CvBridge
import cv2
import numpy as np

from pupil_apriltags import Detector

from rclpy.qos import QoSProfile, ReliabilityPolicy

# ── ANSI 颜色常量 ──────────────────────────────────────────────
_RESET  = '\033[0m'
_BOLD   = '\033[1m'
_GREEN  = '\033[92m'   # 亮绿  —— 检测到 tag
_YELLOW = '\033[93m'   # 亮黄  —— 数量变化提示
_CYAN   = '\033[96m'   # 亮青  —— 每个 tag 的详细信息
_RED    = '\033[91m'   # 亮红  —— 无 tag / 丢失
_BLUE   = '\033[94m'   # 亮蓝  —— 节点启动


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])
        self.bridge   = CvBridge()
        self.detector = Detector()

        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs   = None

        # Tag 尺寸（根据 Gazebo 模型调整）
        self.tag_size = 0.5

        # ── 状态追踪：只在数量变化时重新打印 ────────────────────
        self._last_tag_count = -1       # -1 表示从未输出过
        self._last_tag_ids: set = set() # 上一帧检测到的 tag id 集合

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/camera_sensor/image_raw',
            self.image_callback,
            qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_sensor/camera_sensor/camera_info',
            self.camera_info_callback,
            qos
        )

        self.pub = self.create_publisher(
            TransformStamped,
            '/tag_detections',
            10
        )

        self.get_logger().info(
            f'\n{_BLUE}{_BOLD}'
            f'╔══════════════════════════════════════════════════╗\n'
            f'║    AprilTag Detector Started                     ║\n'
            f'║    PnP frame : camera_optical_link               ║\n'
            f'╚══════════════════════════════════════════════════╝'
            f'{_RESET}'
        )

    # ------------------------------------------------------------------ #

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs   = np.array(msg.d)

    # ------------------------------------------------------------------ #

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        img  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(gray)
        count   = len(results)
        tag_ids = {r.tag_id for r in results}

        # ── 只在「数量」或「id 集合」发生变化时打印 ──────────────
        if count != self._last_tag_count or tag_ids != self._last_tag_ids:
            self._last_tag_count = count
            self._last_tag_ids   = tag_ids
            self._print_detection_summary(count, results, msg)

        # ── 始终正常发布 ──────────────────────────────────────────
        for r in results:
            self._publish_tag(r, msg)

    # ------------------------------------------------------------------ #

    def _print_detection_summary(self, count, results, msg):
        """数量变化时打印彩色摘要。"""

        if count == 0:
            self.get_logger().info(
                f'\n{_RED}{_BOLD}'
                f'┌─────────────────────────────────┐\n'
                f'│  ✗  No AprilTags detected        │\n'
                f'└─────────────────────────────────┘'
                f'{_RESET}'
            )
            return

        color  = _GREEN if count == self._last_tag_count else _YELLOW
        header = (
            f'\n{color}{_BOLD}'
            f'┌─────────────────────────────────────────────────┐\n'
            f'│  ✔  Detected {count} AprilTag{"s" if count > 1 else " "}'
            f'{"  " if count < 10 else " "}'
            f'                      │\n'
            f'├─────────────────────────────────────────────────┤'
            f'{_RESET}'
        )
        self.get_logger().info(header)

        for r in results:
            tag_id = r.tag_id

            half_size     = self.tag_size / 2.0
            image_points  = np.array(r.corners, dtype=np.float32)
            object_points = np.array([
                [-half_size, -half_size, 0],
                [ half_size, -half_size, 0],
                [ half_size,  half_size, 0],
                [-half_size,  half_size, 0],
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_EPNP
            )

            if success:
                tx, ty, tz = tvec.flatten()
                dist = float(np.linalg.norm(tvec))
                self.get_logger().info(
                    f'{_CYAN}'
                    f'\n│  Tag ID      : {tag_id}\n'
                    f'│  Pos(optical): x={tx:+.3f}  y={ty:+.3f}  z={tz:+.3f}  (m)\n'
                    f'│  Dist        : {dist:.3f} m'
                    f'{_RESET}'
                )
            else:
                self.get_logger().info(
                    f'{_CYAN}│  Tag ID : {tag_id}   (PnP solve failed){_RESET}'
                )

        self.get_logger().info(
            f'\n{color}{_BOLD}'
            f'\n└─────────────────────────────────────────────────┘'
            f'{_RESET}'
        )

    # ------------------------------------------------------------------ #

    def _publish_tag(self, r, msg):
        """
        对单个检测结果做 PnP 并发布 TransformStamped。

        PnP 在图像（光学）坐标系下求解，结果天然属于 camera_optical_link。
        frame_id 声明为 camera_optical_link，localizer 侧查询
        camera_optical_link → base_footprint 的 TF 才能正确对齐。

        修复说明：
          原来 frame_id 错误地写成 camera_link，跳过了
          camera_optical_link → camera_link 之间约 -90° 的旋转，
          导致 yaw 误差接近 -90°，机器人方向估计完全错误。
        """

        tag_id        = r.tag_id
        half_size     = self.tag_size / 2.0
        image_points  = np.array(r.corners, dtype=np.float32)
        object_points = np.array([
            [-half_size, -half_size, 0],
            [ half_size, -half_size, 0],
            [ half_size,  half_size, 0],
            [-half_size,  half_size, 0],
        ], dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_EPNP
        )

        if not success:
            return

        R, _ = cv2.Rodrigues(rvec)

        T         = np.eye(4)
        T[:3, :3] = R
        T[:3, 3]  = tvec.flatten()

        quat = self._rotation_matrix_to_quaternion(R)

        t                         = TransformStamped()
        t.header.stamp            = msg.header.stamp
        # FIX: PnP 在光学坐标系下求解，frame_id 必须为 camera_optical_link
        t.header.frame_id         = 'camera_optical_link'
        t.child_frame_id          = f'tag_{tag_id}'
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        t.transform.rotation.x    = quat[0]
        t.transform.rotation.y    = quat[1]
        t.transform.rotation.z    = quat[2]
        t.transform.rotation.w    = quat[3]

        self.pub.publish(t)

    # ------------------------------------------------------------------ #

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
        """将 3×3 旋转矩阵转换为四元数 [x, y, z, w]。"""

        q     = np.zeros(4)
        trace = np.trace(R)

        if trace > 0:
            s    = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s    = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s    = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s    = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

        return q


# ────────────────────────────────────────────────────────────────────────── #

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()