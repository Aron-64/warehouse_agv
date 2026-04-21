# 🚗 仓储AGV多传感器融合定位与自主导航系统

## 📌 项目简介
本项目基于 **ROS 2 + Gazebo 仿真平台**，构建了一套面向仓储场景的 **AGV（自动导引车）多传感器融合定位与自主导航系统**。

系统围绕“定位鲁棒性与精度提升”，融合多源传感器，实现复杂环境下稳定导航。

---

## 🎯 核心功能
- 激光雷达定位（AMCL）
- AprilTag视觉定位（全局校正）
- 轮式里程计（Odometry）
- IMU姿态解算
- EKF多传感器融合
- Nav2自主导航

---

## 🧠 系统架构
```
warehouse_agv/
└── catkin_ws/src/
    ├── bot_description
    ├── bot_gazebo
    ├── bot_apriltag
    ├── bot_localization
    └── bot_navigation
```

---

## 🔄 多传感器融合逻辑

### 局部定位（高频）
- EKF融合 Odom + IMU
- 输出连续位姿

### 全局校正（低频）
- AMCL + AprilTag
- 修正累计误差

---

## 🧭 TF坐标系
```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── laser_link
                ├── imu_link
                └── camera_link
                     └── camera_optical_frame
```

说明：
- map→odom：全局校正
- odom→base_link：EKF输出

---

## 📈 实验分析工具

### Analyse_experiment.py
用于分析定位性能：

- 轨迹对比
- 误差计算（RMSE）
- 多算法对比
- 数据可视化

运行：
```
python3 Analyse_experiment.py
```

---

## ⚙️ 环境依赖
- Ubuntu 20.04 / 22.04
- ROS 2（Foxy / Humble）
- Gazebo
- OpenCV

---

## 🛠️ 编译
```
git clone https://github.com/Aron-64/warehouse_agv.git
cd warehouse_agv/catkin_ws
colcon build
source install/setup.bash
```

---

## 🚀 运行
```
ros2 launch bot_gazebo gazebo_sim.launch.py
ros2 launch bot_localization localization.launch.py
ros2 launch bot_navigation nav2.launch.py
```

---

## 📊 项目特点
- 多传感器融合定位
- 支持视觉+激光+IMU
- 高鲁棒性定位
- 完整导航链路

---

## 👤 作者
Aron-64

## 📄 许可证
仅供学习与科研使用
