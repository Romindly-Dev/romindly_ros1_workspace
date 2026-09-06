# Romindly ROS1 学习套件 · 文档索引

面向客户工程师的 ROS1 (Noetic) 实战教程，运行环境统一为 x86_64 边缘计算单元 + Ubuntu 20.04，工作空间路径统一为 `~/ws_romindly`。

## 学习路线

| 章节 | 内容 | 状态 |
| --- | --- | --- |
| [00 环境搭建](00_环境搭建/01_Ubuntu20.04_ROS_Noetic安装.md) | Noetic 安装、工作空间与代码拉取、常用工具速查 | ✅ |
| [10 ROS1 基础](10_ROS1基础/01_核心概念总览.md) | 核心概念总览、话题通信、launch 与参数服务器 | ✅ |
| [15 仿真入门](15_仿真入门/README.md) | TurtleBot3 三件套 + Gazebo：安装模型、键盘遥控、传感器话题观察 | 🚧 |
| [20 2D 建图](20_2D建图/README.md) | gmapping / hector / cartographer / slam_toolbox 对比、录包建图、地图保存 | 🚧 |
| [25 3D 激光 SLAM](25_3D激光SLAM/README.md) | A-LOAM 原理、LIO-SAM、FAST-LIO 实战与算力评估 | 🚧 |
| [30 定位](30_定位/README.md) | AMCL 原理与调参、robot_localization EKF 融合、hdl_localization 3D 定位 | 🚧 |
| [40 导航](40_导航/README.md) | move_base 架构、costmap 调参、DWA vs TEB、导航调优清单 | 🚧 |
| [45 传感器驱动](45_传感器驱动/README.md) | rplidar / velodyne / realsense 接入、udev 规则固定设备名 | 🚧 |
| [50 部署运维](50_部署运维/README.md) | systemd 自启、Docker 部署、多机通信、性能优化 | 🚧 |

状态说明：✅ 已完成可直接跟做；🚧 编写中，README 列有该章规划小节。

## 新手推荐路径

按编号顺序学即可：先完成 **00 环境搭建**（装好 Noetic、拉齐代码、跑通 `roslaunch urdf_demo display.launch`），再过 **10 ROS1 基础** 建立话题/节点/launch 的概念，然后进入 **15 仿真入门**——之后的建图（20）、定位（30）、导航（40）都在这套仿真环境上做实验，学会后再迁移到实体机器人（45 传感器驱动），最后用 **50 部署运维** 把系统固化交付。已有 ROS 经验、只关心某个专题的读者可直接跳到对应章节，各章开头均注明前置条件。
