# 🚗 仓储AGV多传感器融合定位与导航系统

## 📌 项目简介
本项目基于 **ROS 2** 与 **Gazebo 仿真平台**，构建了一个面向仓储场景的 **AGV（自动导引车）多传感器融合定位与自主导航系统**。

系统融合多种传感器信息，实现高精度、鲁棒的定位与导航能力，主要包括：
- 激光雷达定位（AMCL）
- AprilTag视觉定位
- 轮式里程计（Odometry）
- IMU惯性测量
- EKF多传感器融合

适用于仓储物流机器人、室内导航与定位算法研究等场景。

---

## 🧠 系统架构

本系统采用模块化设计，基于ROS 2功能包划分如下：

```
warehouse_agv/
└── catkin_ws/src/
    ├── bot_description     # 机器人建模（URDF/Xacro）
    ├── bot_gazebo          # Gazebo仿真环境
    ├── bot_apriltag        # AprilTag视觉定位
    ├── bot_localization    # EKF融合定位
    └── bot_navigation      # ROS2导航（Nav2）
```

---

## 🔧 功能模块说明

### 🔹 bot_description（机器人建模）
- 使用 URDF/Xacro 描述机器人结构
- 定义：
  - 机器人几何结构
  - 运动学关系
  - 传感器安装位置（激光雷达、相机、IMU）

### 🔹 bot_gazebo（仿真环境）
- 构建仓储仿真环境（warehouse.world）
- 在Gazebo中加载机器人模型
- 提供仿真启动文件

### 🔹 bot_apriltag（视觉定位）
- 基于 AprilTag 实现视觉定位
- 核心功能：
  - 标签检测
  - 位姿估计（PnP）
  - 坐标变换（相机坐标系 → 地图坐标系）

### 🔹 bot_localization（融合定位）
- 基于 EKF（扩展卡尔曼滤波）实现多传感器融合
- 输入：
  - 里程计（odom）
  - IMU数据
  - AprilTag视觉位姿
- 输出：
  - 优化后的机器人位姿

### 🔹 bot_navigation（导航系统）
- 基于 ROS2 Nav2 框架
- 功能包括：
  - AMCL定位
  - 全局路径规划
  - 局部路径规划
  - Waypoint导航


---

## ⚙️ 环境依赖

- Ubuntu 20.04 / 22.04
- ROS 2（Foxy / Humble）
- Gazebo
- OpenCV
- AprilTag库

---

## 🛠️ 编译与安装

```bash
git clone https://github.com/Aron-64/warehouse_agv.git
cd warehouse_agv/catkin_ws
colcon build
source install/setup.bash
```

---

## 🚀 使用方法

```bash
# 启动仿真
ros2 launch bot_gazebo gazebo_sim.launch.py

# 启动视觉定位
ros2 run bot_apriltag apriltag_detector
ros2 run bot_apriltag apriltag_localizer
ros2 launch bot_localization ekf.launch.py

# 启动导航
ros2 launch bot_navigation nav2.launch.py
```

---

## 📊 系统特点

- 多传感器融合定位
- 支持视觉+激光+里程计
- EKF优化位姿估计
- Nav2完整导航链路
- Gazebo仿真验证

