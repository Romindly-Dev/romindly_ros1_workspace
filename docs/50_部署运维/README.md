# 50 部署运维

> English version: [README.md](../en/50_deployment/README.md)

本章解决"开发完成之后"的问题：把在开发机上跑通的系统部署到 Romindly Mind 边缘计算单元，做到**上电即跑、环境可复现、长期无人值守可运维**。这是面向交付的最后一章。

## 小节列表

| 小节 | 内容 | 配套代码 |
| --- | --- | --- |
| [01 实机部署清单](01_实机部署清单.md) | 从零部署一台边缘单元的完整 checklist：系统设置、ROS 安装（含 apt key 过期修复）、chrony 时间同步、工作空间部署两条路线、install space、多机网络配置、验收测试 | — |
| [02 systemd 开机自启](02_systemd开机自启.md) | 逐行讲解 `ros_env.sh` 与 `ros-robot.service`，安装启用、journalctl 排障、双服务架构、优雅停机 | [romindly_robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) |
| [03 Docker 部署](03_Docker部署.md) | Noetic EOL 后用镜像固化依赖：Dockerfile 逐段讲解、设备/网络/显示透传、离线分发、容器自启、compose 编排 | [docker/](https://github.com/Romindly-Dev/romindly_ros1_workspace/blob/main/docker/README.md) |
| [04 性能优化与运维](04_性能优化与运维.md) | 资源监控与瓶颈定位、按模块降载手段汇总、CPU 绑核、日志治理、远程运维与黑匣子、EOL 下的升级策略 | — |

## 开发机 vs 边缘单元：部署差异

同一套代码，两种环境的运行假设完全不同。部署前先对齐这些差异，本章各小节分别解决对应问题：

| 维度 | 开发机 | 边缘单元（Romindly Mind） | 对应小节 |
| --- | --- | --- | --- |
| 图形界面 | 有桌面，rviz/rqt 随手开 | 通常无显示器（headless），rviz 靠远程或不装 | 01 / 04 |
| 启动方式 | 人工开终端 `roslaunch` | 上电自动启动，无人值守，崩溃要自动重启 | 02 / 03 |
| 电源 | 稳定，随手休眠无所谓 | 现场供电，**必须关闭自动休眠/挂起** | 01 |
| 网络 | DHCP，IP 随时变 | 静态 IP，多机 ROS 通信依赖固定地址与主机名解析 | 01 |
| 软件环境 | 常装常改，坏了重装 | 部署后冻结，依赖必须可复现（EOL 更是如此） | 01 / 03 |
| 系统更新 | 自动更新无所谓 | 自动更新可能半夜拉包重启，**必须关闭** | 01 |
| 时间 | NTP 自动同步 | 可能无外网，多机/多传感器时间戳要 chrony 内网同步 | 01 |
| 日志 | 随手看终端 | 只有 journalctl / rosout 文件，且会日积月累爆盘 | 02 / 04 |

## 前置条件

- 完成 00–45 章：系统功能已在开发环境（仿真或实验台架）验证通过。
- [45 章 udev 规则](../45_传感器驱动/04_udev规则与设备管理.md)已配好，设备名固定为 `/dev/rplidar`、`/dev/base_serial` 等——开机自启强依赖固定设备名。
- 一台待部署的边缘单元（Ubuntu 20.04 x86_64），可通过显示器或 ssh 访问。

## 建议阅读顺序

按 01 → 02 → 03 顺序做完，机器即达到"上电即跑"的交付状态；04 在跑起来之后按需查阅。只用 Docker 交付的读者可跳过 01 的 ROS 裸机安装段，但网络、时间同步与验收清单仍然适用。
