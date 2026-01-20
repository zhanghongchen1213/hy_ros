---
name: "ros-document-generation"
description: "专为机器人项目（ROS/ROS2）设计。自动生成包含节点图、话题交互、TF坐标系、URDF模型及Launch启动流程的技术文档。"
---

# ROS 机器人开发文档专家 (ROS Document Generator)

本技能专注于 ROS/ROS2 机器人工程的文档生成。通过分析 `package.xml`, `CMakeLists.txt`, `.launch.py`, `.msg/.srv` 等文件，自动构建系统拓扑和数据流向图。

## 目标

1.  **架构可视化 (Architecture Visualization)**：理清节点 (Node) 间的发布/订阅关系，构建数据流图。
2.  **接口标准化 (Interface Standardization)**：解析自定义的消息 (`msg`)、服务 (`srv`) 和动作 (`action`) 定义。
3.  **坐标系分析 (TF Analysis)**：梳理机器人连杆 (`Link`) 和关节 (`Joint`) 的 TF 树结构。
4.  **规范输出**：输出**全中文** Markdown 文档。

## 文档模板

````markdown
# [项目名称] 机器人系统开发文档

> ROS 版本：[如：ROS2 Humble / ROS1 Noetic]
> 仿真环境：[如：Gazebo / Ignition]
> 核心功能：[如：SLAM 建图导航 / 机械臂抓取]

## 1. 系统架构 (System Architecture)

### 1.1 功能模块划分

- **感知 (Perception)**：[列出雷达、相机驱动及处理节点]
- **定位 (Localization)**：[如：AMCL, Robot_Localization]
- **规划 (Planning)**：[如：Nav2, MoveIt]
- **控制 (Control)**：[如：Diff_Drive_Controller]

### 1.2 节点关系图 (Node Graph)

> _（描述核心数据流向）_
> `Lidar_Driver` -> `/scan` -> `SLAM_Node` -> `/map` -> `Nav_Node` -> `/cmd_vel` -> `Base_Controller`

## 2. 软件包清单 (Packages)

### 2.1 [包名 A，如：robot_navigation]

- **路径**：`src/robot_navigation`
- **依赖**：`nav2_bringup`, `slam_toolbox`
- **功能**：负责全局路径规划和局部避障

### 2.2 [包名 B，如：robot_description]

- **路径**：`src/robot_description`
- **功能**：包含 URDF 模型、Mesh 文件和 RViz 配置

## 3. 通信接口详解 (Communication Interfaces)

### 3.1 核心话题 (Topics)

| 话题名称   | 消息类型                | 发布者          | 订阅者      | 描述                |
| ---------- | ----------------------- | --------------- | ----------- | ------------------- |
| `/scan`    | `sensor_msgs/LaserScan` | lidar_node      | slam_node   | 2D 激光雷达原始数据 |
| `/odom`    | `nav_msgs/Odometry`     | base_controller | ekf_node    | 轮式里程计          |
| `/cmd_vel` | `geometry_msgs/Twist`   | nav2_controller | base_driver | 速度控制指令        |

### 3.2 自定义服务 (Services)

- **服务名**：`/reset_odometry`
- **类型**：`std_srvs/Trigger`
- **功能**：重置里程计坐标为零点

### 3.3 参数配置 (Parameters)

| 节点        | 参数名              | 默认值 | 说明                           |
| ----------- | ------------------- | ------ | ------------------------------ |
| `amcl`      | `max_particles`     | 2000   | 最大粒子数，影响定位精度与性能 |
| `move_base` | `xy_goal_tolerance` | 0.2    | 到达目标点的允许误差范围 (m)   |

## 4. 坐标系与模型 (TF & URDF)

### 4.1 TF 树结构

`map` -> `odom` -> `base_link` -> `laser_link` / `camera_link`

### 4.2 关键变换

- **odom -> base_link**：由里程计实时发布，反映机器人运动。
- **base_link -> laser_link**：静态变换 (Static TF)，描述雷达安装位置。

## 5. 启动与运行 (Launch & Run)

### 5.1 启动文件说明

- **`bringup.launch.py`**：启动底盘驱动、雷达和 TF 变换。
- **`navigation.launch.py`**：启动 Nav2 协议栈和 RViz。

### 5.2 运行指令

```bash
# 1. 编译工作空间
colcon build --symlink-install

# 2. Source 环境
source install/setup.bash

# 3. 启动机器人
ros2 launch my_robot_bringup robot.launch.py

# 4. 启动键盘控制 (可选)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
````

## 6. 常见问题 (Troubleshooting)

- **现象**：RViz 中雷达数据显示异常。
- **排查**：检查 TF 树是否断裂 (`ros2 run tf2_tools view_frames`)。

```

```
