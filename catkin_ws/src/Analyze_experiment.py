#!/usr/bin/env python3
"""
plot_trajectories.py
────────────────────
从三组实验的 rosbag 中提取轨迹，绘制在同一张图上，
同时画出由目标点连成的理想矩形路径。

依赖：
    pip install rosbags matplotlib numpy

用法：
   python3 Analyze_experiment.py \
       --bags /path/to/group_A /path/to/group_B /path/to/group_C \
       --output trajectory_comparison.png
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
# ── 中文字体修复：在 import pyplot 之前设置 ──────────────────────────
matplotlib.rcParams['font.family']       = 'Noto Sans CJK JP'
matplotlib.rcParams['axes.unicode_minus'] = False   # 负号正常显示

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    sys.exit("请先安装：pip install rosbags")


# ── 理想路径：目标点坐标（与 waypoint_navigator.py 保持一致）────────
WAYPOINTS = [
    ( 2.0, -1.0),
    ( 2.0, -5.0),
    (-0.5, -5.0),
    (-0.5, -1.0),
    ( 2.0, -1.0),   # 回到起点，形成闭合矩形
]

# ── 三组样式 ──────────────────────────────────────────────────────────
GROUP_STYLE = {
    'A': dict(color='#E24B4A', lw=1.6, ls='--',  label='A  纯里程计'),
    'B': dict(color='#378ADD', lw=1.6, ls='-.',  label='B  EKF + AMCL'),
    'C': dict(color='#1D9E75', lw=2.2, ls='-',   label='C  EKF + AMCL + AprilTag'),
}

# 从 bag 中按优先级尝试读取的位姿 topic
POSE_PRIORITY = [
    '/amcl_pose',             # PoseWithCovarianceStamped  ← 首选
    '/odometry/filtered_map', # Odometry
    '/odometry/filtered',     # Odometry
    '/odom',                  # Odometry（最后兜底）
]


# ══════════════════════════════════════════════════════════════════ #

def read_trajectory(bag_path: str):
    """
    从 rosbag 读取轨迹，返回 (xs, ys, topic_used)。
    自动识别 PoseWithCovarianceStamped 和 Odometry 两种消息类型。
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    path      = Path(bag_path).expanduser()

    if not path.exists():
        raise FileNotFoundError(f"找不到 bag：{path}")

    xs, ys     = [], []
    used_topic = None

    with Reader(path) as reader:
        available = {c.topic for c in reader.connections}

        for candidate in POSE_PRIORITY:
            if candidate in available:
                used_topic = candidate
                break

        if used_topic is None:
            raise RuntimeError(f"{bag_path} 中没有可用的位姿 topic")

        conns = [c for c in reader.connections if c.topic == used_topic]

        for conn, _, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            pos = msg.pose.pose.position
            xs.append(pos.x)
            ys.append(pos.y)

    if not xs:
        raise RuntimeError(f"{bag_path} 的 {used_topic} 没有数据")

    return np.array(xs), np.array(ys), used_topic


# ══════════════════════════════════════════════════════════════════ #

def plot(group_trajs: dict, output: str):
    """
    group_trajs: {'A': (xs, ys), 'B': ..., 'C': ...}
    """
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_aspect('equal', 'box')
    ax.set_title('三组定位方案轨迹对比', fontsize=14, fontweight='bold', pad=14)
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.35)

    # ── 理想路径（目标点连线）─────────────────────────────────────
    wx = [p[0] for p in WAYPOINTS]
    wy = [p[1] for p in WAYPOINTS]
    ax.plot(wx, wy,
            color='#333333', linewidth=1.2, linestyle=':',
            zorder=2, label='理想路径（目标点连线）')

    # 标注目标点编号（去掉重复的终点）
    for i, (x, y) in enumerate(WAYPOINTS[:-1]):
        ax.scatter(x, y, s=55, color='#333333', zorder=5,
                   edgecolors='white', linewidths=0.8)
        ax.text(x + 0.08, y + 0.12, f'P{i+1}',
                fontsize=8.5, color='#333333', fontweight='bold')

    # ── 各组实际轨迹 ──────────────────────────────────────────────
    for group in sorted(group_trajs.keys()):
        xs, ys = group_trajs[group]
        st     = GROUP_STYLE.get(group, {})
        ax.plot(xs, ys,
                color=st.get('color', 'gray'),
                linewidth=st.get('lw', 1.5),
                linestyle=st.get('ls', '-'),
                alpha=0.85,
                zorder=3)
        # 起点圆圈
        ax.scatter(xs[0], ys[0],
                   s=50, color=st.get('color', 'gray'),
                   zorder=6, edgecolors='white', linewidths=0.8)

    # ── 图例 ──────────────────────────────────────────────────────
    legend_elements = [
        Line2D([0], [0], color='#333333', lw=1.2, ls=':',
               label='理想路径（目标点连线）'),
    ]
    for group in sorted(group_trajs.keys()):
        st = GROUP_STYLE.get(group, {})
        legend_elements.append(
            Line2D([0], [0],
                   color=st.get('color', 'gray'),
                   lw=st.get('lw', 1.5),
                   ls=st.get('ls', '-'),
                   label=st.get('label', f'组{group}'))
        )

    ax.legend(handles=legend_elements,
              loc='upper right', fontsize=9.5,
              framealpha=0.9, edgecolor='#cccccc')

    # ── 坐标轴范围留边距 ──────────────────────────────────────────
    all_x = [p[0] for p in WAYPOINTS]
    all_y = [p[1] for p in WAYPOINTS]
    for xs, ys in group_trajs.values():
        all_x.extend(xs.tolist())
        all_y.extend(ys.tolist())
    margin = 0.5
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    plt.tight_layout()
    plt.savefig(output, dpi=150, bbox_inches='tight')
    print(f'[✓] 轨迹图已保存：{output}')
    plt.show()


# ══════════════════════════════════════════════════════════════════ #

def main():
    parser = argparse.ArgumentParser(description='实验轨迹对比绘图')
    parser.add_argument(
        '--bags', nargs='+', required=True,
        help='rosbag 路径，按 A B C 顺序传入（可少于三个）'
    )
    parser.add_argument(
        '--groups', nargs='+', default=None,
        help='对应的组别标识，默认按 A B C 顺序自动分配'
    )
    parser.add_argument(
        '--output', default='trajectory_comparison.png',
        help='输出图片路径'
    )
    args = parser.parse_args()

    # 自动分配组别
    all_groups = ['A', 'B', 'C']
    groups = args.groups if args.groups else all_groups[:len(args.bags)]

    if len(groups) != len(args.bags):
        sys.exit('--bags 和 --groups 数量必须一致')

    # ── 读取各组轨迹 ──────────────────────────────────────────────
    group_trajs = {}
    for group, bag_path in zip(groups, args.bags):
        print(f'[→] 读取组 {group}：{bag_path}')
        try:
            xs, ys, topic = read_trajectory(bag_path)
            group_trajs[group] = (xs, ys)
            print(f'    topic : {topic}')
            print(f'    点数  : {len(xs)}')
        except Exception as e:
            print(f'    [!] 跳过，原因：{e}')

    if not group_trajs:
        sys.exit('所有 bag 读取失败，退出。')

    plot(group_trajs, args.output)


if __name__ == '__main__':
    main()