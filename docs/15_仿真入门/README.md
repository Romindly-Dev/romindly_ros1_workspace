# 15 仿真入门

> English version: [README.md](../en/15_simulation/README.md)

为什么先学仿真：没有实体机器人也能在 Gazebo 里完整走通**建图 → 定位 → 导航**全流程，后续 20/30/40 章的所有实验都在这套 TurtleBot3 仿真环境上进行，学会后再原样迁移到实机。

## 本章内容

| 小节 | 内容 | 你将学会 |
| --- | --- | --- |
| [01 TurtleBot3 与 Gazebo 安装](01_TurtleBot3与Gazebo安装.md) | Gazebo 验证、TB3 三件套安装（apt 推荐）、`TURTLEBOT3_MODEL` 环境变量、首次启动仿真世界 | 跑通 `turtlebot3_world.launch` |
| [02 键盘遥控与话题观察](02_键盘遥控与话题观察.md) | teleop 键盘遥控；用 rostopic/rqt_graph/RViz 观察 `/scan` `/odom` `/cmd_vel` `/imu` | 看懂仿真机器人的数据流 |
| [03 仿真中的 TF 树与传感器](03_仿真中的TF树与传感器.md) | view_frames 导出 TF 树并逐层讲解；robot_state_publisher 与 Gazebo 插件的分工；`/clock` 与 `use_sim_time` | 理解建图算法需要的 `/scan` + TF 从哪来 |

## 前置条件

- 完成 [00 环境搭建](../00_环境搭建/01_Ubuntu20.04_ROS_Noetic安装.md)（desktop-full 已含 Gazebo 11）
- 学完 [10 ROS1 基础](../10_ROS1基础/README.md)，尤其是 [02 话题通信](../10_ROS1基础/02_话题通信.md)、[06 TF2 坐标变换](../10_ROS1基础/06_TF2坐标变换.md)、[07 URDF 机器人建模](../10_ROS1基础/07_URDF机器人建模.md)、[08 rosbag 与调试工具](../10_ROS1基础/08_rosbag与调试工具.md)——本章会反复回扣这几篇

学完本章即可进入 [20 2D 建图](../20_2D建图/README.md)。
