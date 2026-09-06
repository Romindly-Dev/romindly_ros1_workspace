# 50 部署运维

> 状态：🚧 编写中

本章解决"开发完成之后"的问题：让机器人上电即跑、环境可复现、多机可通信、资源占用可控，面向交付与长期运行场景。

## 规划小节

- 01 systemd 自启：为 roslaunch 编写 service 单元、开机自启、日志查看（journalctl）与故障自动重启
- 02 Docker 部署：Noetic EOL 后用镜像固化环境、宿主机设备/网络/显示透传、docker-compose 编排（配合 [docker/README.md](../../docker/README.md)）
- 03 多机通信：ROS_MASTER_URI / ROS_IP（ROS_HOSTNAME）配置、时间同步（chrony）、常见"能 list 不能 echo"问题
- 04 性能优化：nodelet 减少拷贝、话题频率与队列控制、CPU 绑核、日志级别与磁盘占用治理
- 05 交付检查清单：上电自检、网络恢复、异常重启策略

## 前置条件

完成前序功能章节，系统功能已在开发环境验证通过。
