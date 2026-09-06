# 40 导航

> 状态：🚧 编写中

本章围绕 ROS1 导航栈（navigation stack）展开，从 move_base 架构到可直接套用的调优清单，目标是让机器人在真实环境中稳定地从 A 点走到 B 点。

## 规划小节

- 01 move_base 架构：全局/局部规划器、costmap、recovery 行为的协作流程图解
- 02 costmap 调参：static / obstacle / inflation 三层参数详解，footprint 与 inflation_radius 的关系
- 03 全局规划器：navfn 与 global_planner 的差异
- 04 局部规划器对比：DWA vs TEB —— 原理、参数、适用底盘（差速/全向/阿克曼）
- 05 导航调优清单：速度限幅、容忍度、震荡与卡死问题的逐项排查表
- 06 实战：仿真环境完整导航 demo 与真实机器人迁移注意事项

## 前置条件

完成 [30 定位](../30_定位/README.md)——导航依赖稳定的 map→odom→base_link TF 链。
