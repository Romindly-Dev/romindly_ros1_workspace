# 45 传感器驱动

> English version: [README.md](../en/45_sensor_drivers/README.md)

前面的章节（20 2D 建图、25 3D 激光 SLAM、30 定位、40 导航）大多可以在仿真或公开数据集上完成。本章解决"从仿真走向实机"的第一步：把真实传感器接到边缘计算单元 **Romindly Mind**（Ubuntu 20.04 / ROS Noetic / x86_64）上，装好驱动、验证话题、固定设备名，为跑通实机建图定位导航打好地基。

## 本章内容

| 小节 | 传感器 / 主题 | 关键产出 |
|------|--------------|----------|
| [01 RPLIDAR 接入](01_RPLIDAR接入.md) | 思岚 2D 激光雷达（A1/A2/A3/S/C 系列） | `/scan` 话题，接 20 章 gmapping / slam_toolbox |
| [02 Velodyne 与 Livox 接入](02_Velodyne与Livox接入.md) | 3D 机械式（VLP-16）与 3D 固态（Livox） | `/velodyne_points`、`/livox/lidar`，接 25 章 3D SLAM |
| [03 RealSense 深度相机接入](03_RealSense深度相机接入.md) | Intel RealSense D400 系列 | 彩色/深度/点云/IMU 话题，接 40 章 costmap 障碍层 |
| [04 udev 规则与设备管理](04_udev规则与设备管理.md) | 串口设备名固定、网口静态 IP、开机自启前置 | `/dev/rplidar` 等稳定设备名，呼应 [robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) |

## 传感器选型速查表

| 类型 | 代表型号 | 接口方式 | 数据形态 | 适用章节 | 本章小节 |
|------|----------|----------|----------|----------|----------|
| 2D 激光雷达 | 思岚 RPLIDAR A1/A2/A3/S/C | USB 串口（CP210x 转 `/dev/ttyUSB*`） | `sensor_msgs/LaserScan` | [20 2D 建图](../20_2D建图/README.md)、[30 定位](../30_定位/README.md)、[40 导航](../40_导航/README.md) | 01 |
| 3D 机械式激光雷达 | Velodyne VLP-16 | 以太网（UDP，雷达默认 192.168.1.201） | `sensor_msgs/PointCloud2` | [25 3D 激光 SLAM](../25_3D激光SLAM/README.md)（A-LOAM / LIO-SAM / FAST-LIO2） | 02 |
| 3D 固态激光雷达 | Livox Horizon / Avia / Mid-40 | 以太网（按广播码识别） | `PointCloud2` 或 Livox `CustomMsg` | [25 3D 激光 SLAM](../25_3D激光SLAM/README.md)（FAST-LIO2 首选） | 02 |
| 深度相机 | Intel RealSense D435i / D455 | USB 3.x（Type-C 接口） | 彩色/深度图像、点云、IMU | [40 导航](../40_导航/README.md)（costmap 障碍层）、视觉类扩展 | 03 |

选型经验法则：

- **室内小车、预算有限、先跑通 2D 流程** → RPLIDAR（01 节），配 gmapping / slam_toolbox 足够。
- **室外或大场景、需要 3D 点云地图** → VLP-16 或 Livox（02 节）；有 IMU 且追求低算力占用选 Livox + FAST-LIO2。
- **导航时要躲避低矮/悬空障碍物（2D 雷达扫不到）** → 加一台 RealSense（03 节），深度点云进 costmap。
- 多个串口设备同时上机（雷达 + IMU + 底盘）时，**必读 04 节**，否则重启后 `/dev/ttyUSB0/1` 漂移会让 launch 随机失败。

## 接口资源规划（Romindly Mind）

Romindly Mind 提供双 2.5G 网口、3× Type-C、UART×4、CAN、GPIO。本章的典型占用：

- 网口 1：Velodyne / Livox 雷达（静态 IP，见 02/04 节）；网口 2：外网 / 调试。
- Type-C：RealSense（需 USB3 线缆与接口，见 03 节）；RPLIDAR 经 USB 转接亦走 Type-C。
- UART / CAN：底盘与 IMU（设备名固定见 04 节）。

## 前置条件

- 完成 [00 环境搭建](../00_环境搭建/01_Ubuntu20.04_ROS_Noetic安装.md)，`catkin_ws` 可正常编译。
- 有对应实体传感器；3D 雷达还需自备电源适配器与网线。
