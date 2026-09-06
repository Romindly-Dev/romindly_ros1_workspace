# 02 systemd 开机自启

> English version: [02_systemd_autostart.md](../en/50_deployment/02_systemd_autostart.md)

## 目标

- 读懂 [robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) 提供的 [ros_env.sh](https://github.com/Romindly-Dev/romindly_robot_bringup/blob/main/robot_bringup/scripts/ros_env.sh) 与 [ros-robot.service](https://github.com/Romindly-Dev/romindly_robot_bringup/blob/main/systemd/ros-robot.service)，完成安装启用与日志排障。
- 实操三个改进：等待设备就绪、roscore/应用双服务架构、优雅停机。

## 背景：为什么是 systemd

交付机器的要求是"上电即跑、崩了自己爬起来、日志有处可查"。systemd 是 Ubuntu 自带的服务管理器，正好逐条满足：开机按依赖顺序拉起服务、`Restart=` 自动重启、`journalctl` 统一收日志。相比 `rc.local`/crontab `@reboot` 这类土办法，它能表达"等网络就绪再启动""A 挂了连带重启 B"这类依赖关系。

## 逐行讲解

### ros_env.sh —— 服务的入口脚本

```bash
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/iot/workspace/ws_romindly/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

exec roslaunch robot_bringup robot.launch --wait
```

- **为什么要显式 source？** systemd 启动进程时环境几乎是空的——它**不读 `.bashrc`**（那是交互式 shell 的配置）。你在终端里能 `roslaunch` 是因为 `.bashrc` 帮你 source 过 setup.bash；服务里必须自己来。同理 `ROS_MASTER_URI` 等变量也要在脚本里显式 export。多机部署时把两个 localhost 改成实际 IP（见 [01 节](01_实机部署清单.md)网络段）。
- **`exec`**：用 roslaunch 进程**替换** bash 进程，使 systemd 直接监管 roslaunch——PID 对得上，信号（停止/重启）能直达，不会出现"bash 死了 roslaunch 变孤儿"。
- **`--wait`**：roslaunch 等待 roscore 出现而不是自己起一个，为下文双服务架构做准备（单服务模式下不加也能跑，roslaunch 会自起 master）。
- 注意路径是 `devel/setup.bash`；按 01 节路线 B 部署 install 空间的机器改为 `install/setup.bash`。

### ros-robot.service —— 服务单元

```ini
[Unit]
Description=ROS1 Robot Bringup
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=iot
ExecStart=/home/iot/workspace/ws_romindly/src/romindly_robot_bringup/robot_bringup/scripts/ros_env.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

| 行 | 作用 | 说明 |
| --- | --- | --- |
| `After=network-online.target` | 排序：等"网络在线"再启动 | 只定顺序不定依赖，须与 Wants 搭配 |
| `Wants=network-online.target` | 弱依赖：把该 target 拉进启动集 | 多机部署强烈需要；纯 localhost 也建议保留 |
| `Type=simple` | ExecStart 进程本身就是主进程 | 配合脚本里的 `exec` |
| `User=iot` | 以 iot 用户运行 | 不用 root：串口权限走 dialout 组/udev，日志目录在 `/home/iot/.ros` |
| `ExecStart=` | 入口脚本绝对路径 | systemd 里一律绝对路径 |
| `Restart=on-failure` | 非 0 退出即自动重启 | 崩溃自愈；`systemctl stop` 的正常停止不会触发 |
| `RestartSec=5` | 重启前等 5 秒 | 给设备/网络喘息，避免疯狂重启刷日志 |
| `WantedBy=multi-user.target` | enable 后挂到多用户运行级 | 无桌面的 Server 也会启动 |

## 安装、启用与日志

```bash
cd ~/ws_romindly/src/romindly_robot_bringup
sudo cp systemd/ros-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ros-robot.service   # enable 开机自启 + 立即启动

systemctl status ros-robot        # Active: active (running) 即成功
journalctl -u ros-robot -f        # 实时日志（roslaunch 的输出都在这）
journalctl -u ros-robot -b        # 本次开机以来的全部日志
journalctl -u ros-robot --since "10 min ago" -p err   # 只看最近错误
```

改过 `.service` 文件后必须 `daemon-reload` 再 `restart` 才生效。验收：`sudo reboot`，重启后跑 [01 节验收清单](01_实机部署清单.md) 第 3–6 条。

## 改进一：等待设备就绪

网络就绪不代表雷达就绪。udev 处理慢时驱动会报"打不开 /dev/rplidar"然后靠 Restart 重试——能跑但不优雅。systemd 会为设备自动生成 device unit，路径按 `/` → `-` 转义，直接依赖它（呼应 [45 章 udev 篇](../45_传感器驱动/04_udev规则与设备管理.md)）：

```ini
[Unit]
After=network-online.target dev-rplidar.device
Wants=network-online.target
Requires=dev-rplidar.device
```

`Requires=` 表示设备没出现就不启动、设备消失服务也停。多个设备就多列几个（`dev-base_serial.device` 等）。验证设备 unit 存在：`systemctl status dev-rplidar.device`。

## 改进二：roscore 单独服务（双服务架构）

单服务模式下 roslaunch 崩溃重启会连 master 一起重启，参数服务器上的内容（地图、标定参数）全部丢失，其他机器上的节点也要全部重连。把 roscore 拆出来独立守护更稳。新建 `/etc/systemd/system/ros-core.service`：

```ini
[Unit]
Description=ROS1 Master (roscore)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=iot
ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash && exec roscore"
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

然后让 ros-robot.service 依赖它：

```ini
[Unit]
After=network-online.target ros-core.service dev-rplidar.device
Requires=ros-core.service
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now ros-core ros-robot
```

`ros_env.sh` 里的 `--wait` 此时发挥作用：roslaunch 等 ros-core 的 master 就绪，不再自起。效果：应用崩了只重启应用，master 与参数不动。

## 改进三：优雅停机

roslaunch 收到 **SIGINT**（等价 Ctrl+C）才会走正常关闭流程：逐个通知节点 shutdown、节点执行清理（底盘发停车指令、关串口、写完日志）。而 systemd 默认发 SIGTERM，roslaunch 对它的处理不如 SIGINT 干净。在 `[Service]` 段加：

```ini
KillSignal=SIGINT
TimeoutStopSec=20
```

`TimeoutStopSec` 给节点 20 秒收尾，超时才 SIGKILL 强杀。对"停止服务时底盘必须收到停车命令"的场景这两行是安全项，不是美化项。

## 常见问题

- **开机起太早、节点连不上网**：确认同时写了 `After=` 和 `Wants=network-online.target`，并 `systemctl enable systemd-networkd-wait-online`（netplan+networkd 环境）或 `NetworkManager-wait-online`。仍不稳可加 `ExecStartPre=/bin/sleep 5` 兜底。
- **手动能跑、服务里起不来**：九成是环境问题——记住 systemd 不读 `.bashrc`。对比 `journalctl -u ros-robot` 里的报错，检查脚本 source 的路径（devel vs install）。
- **日志爆盘**：journal 默认可占相当大的磁盘。编辑 `/etc/systemd/journald.conf` 设 `SystemMaxUse=500M`，`sudo systemctl restart systemd-journald`；rosout 文件日志的治理见 [04 节](04_性能优化与运维.md)。
- **`status` 显示 activating (auto-restart) 循环**：服务在崩溃重启环里，`journalctl -u ros-robot -b -p err` 看第一次崩的原因，常见是设备不在（配 `Requires=dev-*.device` 让原因显性化）。
- **改了 service 没生效**：忘了 `daemon-reload`。
