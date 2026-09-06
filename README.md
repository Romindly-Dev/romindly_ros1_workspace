# Romindly ROS1 Workspace

ROS1 (Noetic) 定位·导航·基础 学习与部署套件 —— 面向 x86 边缘计算单元。

**🛸 教程总览主页：[romindly-dev.github.io/romindly_ros1_workspace](https://romindly-dev.github.io/romindly_ros1_workspace/)** · 公司官网：[romindly.com/zh](https://www.romindly.com/zh)

**🌐 双语教程 / Bilingual docs**：中文从 [docs/](docs/) 开始，English starts at [docs/en/](docs/en/README.md)。全部 38 篇教程均有中英两版，文内可互相跳转。

本仓库是整个系列的**总入口**：包含全套中文教程、一键拉取所有代码的 `.repos` 文件、Docker 部署方案与 CI。

## 适用环境

| 项目 | 要求 |
|---|---|
| 硬件 | x86_64 边缘计算单元 / PC |
| 系统 | Ubuntu 20.04 LTS |
| ROS | Noetic Ninjemys |

## 快速开始

```bash
# 1. 创建工作空间并克隆本仓库
mkdir -p ~/ws_romindly/src && cd ~/ws_romindly/src
git clone https://github.com/Romindly-Dev/romindly_ros1_workspace.git

# 2. 用 vcstool 一键拉取全部仓库（首次需: sudo apt install python3-vcstool）
cd ~/ws_romindly
vcs import src < src/romindly_ros1_workspace/ros1.repos

# 3. 安装依赖并编译
rosdep install --from-paths src --ignore-src -r -y
catkin_make

# 4. 验证
source devel/setup.bash
roslaunch urdf_demo display.launch   # 应弹出 RViz 并显示差速小车模型
```

没装 ROS？先看 [docs/00_环境搭建](docs/00_环境搭建/01_Ubuntu20.04_ROS_Noetic安装.md)。

## 教程目录（学习路线）

| 章节 | 内容 | 状态 |
|---|---|---|
| [00 环境搭建](docs/00_环境搭建/) | Ubuntu 20.04 + Noetic 安装、工作空间、常用工具 | ✅ |
| [10 ROS1 基础](docs/10_ROS1基础/) | 话题、服务、Action、TF2、URDF、launch、rosbag | ✅ |
| [15 仿真入门](docs/15_仿真入门/) | TurtleBot3 + Gazebo 仿真环境 | ✅ |
| [20 2D 建图](docs/20_2D建图/) | gmapping / hector / cartographer / slam_toolbox 对比与调参 | ✅ |
| [25 3D 激光 SLAM](docs/25_3D激光SLAM/) | A-LOAM / LIO-SAM / FAST-LIO 实战 | ✅ |
| [30 定位](docs/30_定位/) | AMCL、robot_localization 多传感器融合、hdl_localization | ✅ |
| [40 导航](docs/40_导航/) | move_base 架构、costmap、DWA vs TEB 调参 | ✅ |
| [45 传感器驱动](docs/45_传感器驱动/) | 激光雷达 / 深度相机 / IMU 实机接入 | ✅ |
| [50 部署运维](docs/50_部署运维/) | 实机部署、Docker、开机自启、性能优化 | ✅ |

## 仓库总览

代码仓库均在 [Romindly-Dev](https://github.com/Romindly-Dev) 组织下，fork 仓库保留上游 license 与出处，教程集中在本仓库。

**自建仓库**

| 仓库 | 内容 |
|---|---|
| [romindly_ros1_workspace](https://github.com/Romindly-Dev/romindly_ros1_workspace) | 本仓库：教程 + .repos + Docker + CI |
| [romindly_ros1_tutorials](https://github.com/Romindly-Dev/romindly_ros1_tutorials) | ROS1 基础示例包（7 个可编译运行的包） |
| [romindly_robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) | x86 边缘单元启动包：传感器集成、开机自启 |

**Fork 仓库（保留上游出处）**

| 分类 | 仓库 | 上游 |
|---|---|---|
| 仿真 | turtlebot3 / turtlebot3_msgs / turtlebot3_simulations | ROBOTIS-GIT |
| 基础 | ros_tutorials | ros |
| 2D 建图 | slam_gmapping / openslam_gmapping | ros-perception |
| 2D 建图 | hector_slam | tu-darmstadt-ros-pkg |
| 2D 建图 | cartographer / cartographer_ros | cartographer-project |
| 2D 建图 | slam_toolbox | SteveMacenski |
| 3D SLAM | A-LOAM ¹ | HKUST-Aerial-Robotics |
| 3D SLAM | LIO-SAM ¹ | TixiaoShan |
| 3D SLAM | FAST_LIO ¹ | hku-mars |
| 3D 定位 | hdl_localization / ndt_omp / fast_gicp / hdl_global_localization | koide3 |
| 融合定位 | robot_localization | cra-ros-pkg |
| 导航 | navigation | ros-planning |
| 导航 | teb_local_planner / costmap_converter | rst-tu-dortmund |
| 驱动 | rplidar_ros | Slamtec |
| 驱动 | velodyne | ros-drivers |
| 驱动 | livox_ros_driver | Livox-SDK |
| 驱动 | realsense-ros | IntelRealSense |

> ¹ 这三个 fork 已包含 Noetic 编译适配补丁（C++14 / OpenCV4 / 并行编译依赖修复），上游原版在 Ubuntu 20.04 下无法直接编译——这正是使用本组织 fork 的价值。

## Docker

Noetic 已于 2025 年 5 月停止官方维护，建议生产环境使用本仓库提供的 Docker 镜像锁定依赖：

```bash
cd docker && docker build -t romindly/ros1-noetic .
```

详见 [docker/README.md](docker/README.md)。

## License

- 本仓库（教程与自建代码）：MIT
- Fork 仓库：遵循各自上游 license（BSD / Apache-2.0 / GPL 等，见各仓库）
