# 20 2D 建图

> 状态：🚧 编写中

本章覆盖 2D 激光 SLAM 的主流方案，从原理对比到实际录包建图与地图保存，产出可用于定位与导航章节的 2D 栅格地图。

## 规划小节

- 01 四种方案对比：gmapping / hector_slam / cartographer / slam_toolbox 的原理差异、依赖、适用场景与选型建议
- 02 gmapping 实战：仿真环境建图、关键参数（particles、map_update_interval 等）
- 03 录包建图工作流：rosbag record 采集、离线回放建图、use_sim_time 注意事项
- 04 cartographer 安装与配置（Noetic 源码编译要点）
- 05 slam_toolbox 建图与地图续扫（lifelong mapping）
- 06 地图保存与格式：map_server / map_saver、pgm+yaml 结构解析

## 前置条件

完成 [15 仿真入门](../15_仿真入门/README.md)，或已有可发布 /scan 与 /odom 的实体机器人。
