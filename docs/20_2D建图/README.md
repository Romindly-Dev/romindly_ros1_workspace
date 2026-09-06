# 20 · 2D 激光建图

> English version: [README.md](../en/20_2d_slam/README.md)

本章讲解基于 2D 激光雷达的 SLAM 建图。学完本章后，你应能在仿真与实机上使用四种主流方案完成建图、保存地图，并能根据项目需求做方案选型与基本调参。

## 前置要求

- 已完成 [10_ROS1基础](../10_ROS1基础/README.md) 与 [15_仿真入门](../15_仿真入门/README.md)
- 已安装 TurtleBot3 仿真与 SLAM 相关包：

```bash
sudo apt install ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-slam \
                 ros-noetic-turtlebot3-teleop ros-noetic-map-server
```

- 本章所有演示统一使用 TurtleBot3 Burger + `turtlebot3_world` 仿真环境。

## 章节目录

| 序号 | 文档 | 内容 |
| --- | --- | --- |
| 01 | [建图原理与方案选型](./01_建图原理与方案选型.md) | 占据栅格地图、SLAM 问题定义、三条技术路线、输入要求、选型建议 |
| 02 | [gmapping 实战](./02_gmapping实战.md) | 粒子滤波建图完整流程、参数详解、地图保存与 map.yaml |
| 03 | [hector_slam 实战](./03_hector_slam实战.md) | 无里程计建图、TF 配置要点、快速旋转丢失定位的对策 |
| 04 | [slam_toolbox 实战](./04_slam_toolbox实战.md) | 产品化首选方案、sync/async、posegraph 序列化、纯定位模式 |
| 05 | [cartographer 实战](./05_cartographer实战.md) | lua 配置体系、离线建图、回环调参思路 |

## 四种方案一页对比

| 方案 | 原理类别 | 是否需要里程计 | CPU 占用 | 大场景表现 | 推荐场景 |
| --- | --- | --- | --- | --- | --- |
| gmapping | 粒子滤波（RBPF） | 需要（TF odom→base_link） | 中（随粒子数线性增长） | 差：无回环检测，长走廊/大回环易错位；粒子数不足时地图撕裂 | 里程计较好的小型室内场景；教学入门 |
| hector_slam | 高频扫描匹配（多分辨率高斯牛顿） | 不需要 | 低 | 差：无回环、无里程计约束，几何特征少的长走廊易漂移 | 手持雷达快速扫图、无编码器底盘、高帧率雷达（≥20 Hz） |
| slam_toolbox | 图优化（scan-to-map 前端 + Ceres 后端，带回环） | 需要 | 中 | 好：回环闭合 + 全局优化，支持超大地图序列化续建 | 产品化部署首选；大型仓库/厂房；需要持续建图或纯定位 |
| cartographer | 图优化（子图 + 分支定界回环 + 全局 BA） | 可选（推荐提供） | 中高（后台优化线程） | 很好：子图机制 + 强回环，超大场景与多传感器融合表现最佳 | 超大场景、多传感器（IMU/里程计/多雷达）融合、离线精细建图 |

> 补充说明：karto（`slam_karto`）与 slam_toolbox 同源（slam_toolbox 基于 Karto 前端重写并大幅增强），Noetic 上直接学 slam_toolbox 即可，本章不单独展开。

## 通用约定

- 环境变量：所有终端先执行 `export TURTLEBOT3_MODEL=burger`（建议写入 `~/.bashrc`）。
- 地图统一保存到 `~/maps/` 目录，后续 [30_定位](../30_定位/README.md) 与 [40_导航](../40_导航/README.md) 章节会复用。
- 各文档同时给出两种启动方式：TB3 封装的 `turtlebot3_slam.launch`（教学快捷）与直接启动算法节点的通用写法（实机移植用）。
