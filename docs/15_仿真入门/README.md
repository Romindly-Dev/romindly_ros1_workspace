# 15 仿真入门

> 状态：🚧 编写中

本章介绍在没有实体机器人的情况下，用 TurtleBot3 + Gazebo 搭建完整仿真环境，为后续建图、定位、导航章节提供统一的实验平台。

## 规划小节

- 01 TurtleBot3 三件套安装（turtlebot3 / turtlebot3_msgs / turtlebot3_simulations）与 `TURTLEBOT3_MODEL` 环境变量
- 02 Gazebo 世界启动：empty_world、turtlebot3_world、turtlebot3_house
- 03 键盘遥控：turtlebot3_teleop 使用与速度参数说明
- 04 传感器话题观察：/scan、/odom、/imu、/camera 的 rostopic/RViz 查看方法
- 05 Gazebo 常见问题：模型下载慢（国内源）、GPU/软渲染、实时率过低的处理

## 前置条件

完成 [00 环境搭建](../00_环境搭建/01_Ubuntu20.04_ROS_Noetic安装.md)（desktop-full 已含 Gazebo 11）。
