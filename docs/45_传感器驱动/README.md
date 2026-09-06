# 45 传感器驱动

> 状态：🚧 编写中

本章介绍常用传感器在 Noetic 下的驱动安装、话题验证，以及用 udev 规则固定设备名——多传感器上机后最容易踩的坑。

## 规划小节

- 01 RPLIDAR 系列接入：rplidar_ros 安装、串口权限、/scan 验证
- 02 Velodyne 多线雷达接入：velodyne 驱动、网口配置、点云话题与坐标系
- 03 RealSense 深度相机接入：realsense2_camera（librealsense 安装、国内源）、深度/彩色/IMU 话题
- 04 udev 规则固定设备名：为什么 /dev/ttyUSB0 会漂移、按 VID/PID/序列号写规则、创建 /dev/rplidar 等软链
- 05 传感器时间戳与坐标系：frame_id 规范、static_transform_publisher 挂 TF

## 前置条件

完成 [00 环境搭建](../00_环境搭建/01_Ubuntu20.04_ROS_Noetic安装.md)；有对应的实体传感器。
