# 25 · 3D 激光 SLAM

本章面向多线激光雷达（16 线及以上机械式、Livox 固态）场景，讲解三种代表性 3D SLAM 方案的原理与实战：A-LOAM（纯激光特征法）、LIO-SAM（因子图紧耦合）、FAST-LIO（迭代卡尔曼紧耦合）。学完本章后，你应能在 Romindly Mind 上用公开数据集跑通三套方案，理解各自的传感器要求与参数体系，并能根据硬件条件完成选型。

## 前置要求

- 已完成 [20_2D建图](../20_2D建图/README.md)、[30_定位](../30_定位/README.md)、[40_导航](../40_导航/README.md)（理解 SLAM 问题定义、TF 树、rosbag 回放）
- 无 3D 雷达实机也可学习本章：三套方案均提供官方公开 rosbag，获取方式见 [01 概览与准备](./01_3D激光SLAM概览与准备.md)

## 章节目录

| 序号 | 文档 | 内容 |
| --- | --- | --- |
| 01 | [3D 激光 SLAM 概览与准备](./01_3D激光SLAM概览与准备.md) | 2D vs 3D、技术脉络、点云基础、依赖安装总表（ceres/GTSAM/Livox-SDK）、仓库拉取与公开数据集 |
| 02 | [A-LOAM 实战](./02_A-LOAM实战.md) | LOAM 边缘/平面特征、两级优化、KITTI/NSH 数据集跑通、漂移现象观察 |
| 03 | [LIO-SAM 实战](./03_LIO-SAM实战.md) | 因子图四类因子、9 轴 IMU 与 ring/time 字段要求、params.yaml 详解、savePCD 保存地图 |
| 04 | [FAST-LIO 实战](./04_FAST-LIO实战.md) | 迭代误差状态卡尔曼 + ikd-Tree、velodyne/livox 双数据源、边缘单元部署调优 |

## 三方案一页对比

| 方案 | 传感器要求 | 需要 9 轴 IMU | 回环检测 | CPU 占用 | 地图输出形式 | 推荐场景 |
| --- | --- | --- | --- | --- | --- | --- |
| A-LOAM | 纯激光（多线机械式，如 VLP-16/HDL-32/64） | 不需要 IMU | 无 | 中（Ceres 两级优化，激烈运动时里程计降频） | rviz 中的实时点云地图（不自带保存服务） | 教学入门、理解 LOAM 特征法；平稳低速平台 |
| LIO-SAM | 激光 + IMU（点云必须带 ring/time 字段；IMU ≥200 Hz）| **需要**（依赖 roll/pitch/yaw 初始化姿态） | 有（ICP 回环 + 因子图全局优化，可选 GPS 因子） | 中高（iSAM2 后端 + 回环线程，`numberOfCores` 可调） | 关键帧点云拼接，`savePCD`/`save_map` 服务导出 PCD | 大场景建图、需要回环与 GPS 融合的测绘级任务 |
| FAST-LIO | 激光 + IMU（6 轴即可；原生支持 Livox CustomMsg 与标准 PointCloud2） | 不需要（6 轴够用） | 无 | 低（ikd-Tree 增量式地图，无特征提取默认关闭） | 退出时保存整幅 PCD（`pcd_save_en`） | 算力受限的边缘/机载平台、实时里程计优先的场景 |

> Romindly Mind 配置建议：N150 可跑通三套方案的数据集回放，但 LIO-SAM 开回环后余量不多；**跑 3D SLAM 建议选 N305 款（8 核），实时建图与回环优化更从容**。

## 通用约定

- 工作空间统一为 `~/ws_romindly`，四个相关仓库（A-LOAM、LIO-SAM、FAST_LIO、livox_ros_driver）已随 [00 环境搭建](../00_环境搭建/README.md) 的拉取脚本位于 `src/` 下。
- 演示数据集统一下载到 `~/bags/` 目录。
- 三篇实战均先讲原理、再跑数据集、最后给实机接入要点；数据集回放时**不需要**连接任何实体雷达。
