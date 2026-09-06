# Ubuntu 20.04 安装 ROS Noetic

> English version: [01_install_ubuntu2004_ros_noetic.md](../en/00_setup/01_install_ubuntu2004_ros_noetic.md)

## 目标

在 x86_64 边缘计算单元（Ubuntu 20.04 Focal）上完成 ROS Noetic 的完整安装，包括 apt 源配置、桌面完整版安装、rosdep 初始化（含国内替代方案）、环境变量配置，并用 turtlesim 验证安装成功。

> **注意**：ROS Noetic 已于 2025 年 5 月正式 EOL（停止维护）。apt 源仍然可用，软件包可以正常安装，但不再有安全更新和 bug 修复。生产环境建议配合 Docker 固化运行环境，参见 [Docker 部署说明](../../docker/README.md)。

## 背景简述

ROS Noetic 是 ROS1 的最后一个 LTS 发行版，只支持 Ubuntu 20.04。安装分四步：添加软件源和密钥 → apt 安装 → 初始化 rosdep（依赖解析工具）→ 配置环境变量。国内网络访问官方源较慢，本文同时给出清华镜像源和 rosdepc 替代方案。

## 分步操作

### 1. 确认系统版本

```bash
lsb_release -a
```

预期输出：

```
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.6 LTS
Release:        20.04
Codename:       focal
```

若不是 20.04（focal），Noetic 无法通过 apt 安装，请先解决系统版本问题。

### 2. 添加 ROS apt 源

国内推荐清华镜像源（速度快）：

```bash
sudo sh -c '. /etc/os-release && echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $VERSION_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```

如需使用官方源，将 URL 替换为 `http://packages.ros.org/ros/ubuntu/`。

### 3. 添加密钥

```bash
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

预期输出：

```
OK
```

若 `raw.githubusercontent.com` 无法访问，改用 keyserver：

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 4. 安装 ros-noetic-desktop-full

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
```

desktop-full 包含 ROS 核心、rqt 工具集、RViz、Gazebo 11 和常用感知库，约需下载 500MB+，安装后占用约 3GB 磁盘空间。

验证包已装上：

```bash
apt list --installed 2>/dev/null | grep ros-noetic-desktop-full
```

预期输出：

```
ros-noetic-desktop-full/focal,now 1.5.0-1focal.xxxxxxxx amd64 [installed]
```

### 5. 初始化 rosdep

rosdep 用于解析并安装 ROS 包的系统依赖。官方初始化方式：

```bash
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update --include-eol-distros   # Noetic 已 EOL, 不加此参数将解析不到任何 ROS 包
```

**国内网络下 `rosdep init` 和 `rosdep update` 大概率超时失败**，推荐使用国内社区维护的 rosdepc（"c" 指 China）替代：

```bash
sudo apt install -y python3-pip
pip3 install rosdepc
sudo rosdepc init
rosdepc update
```

预期输出（末尾）：

```
updated cache in /home/<用户名>/.ros/rosdep/sources.cache
```

之后凡是文档中出现 `rosdep` 命令的地方，都可以用 `rosdepc` 等价替换（参数完全相同）。

### 6. 配置环境变量

将 ROS 环境写入 `.bashrc`，每次开终端自动生效：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

验证：

```bash
echo $ROS_DISTRO
```

预期输出：

```
noetic
```

### 7. 验证安装：roscore + turtlesim

打开**第一个终端**，启动 ROS Master：

```bash
roscore
```

预期输出（末尾）：

```
started core service [/rosout]
```

打开**第二个终端**，启动小海龟仿真：

```bash
rosrun turtlesim turtlesim_node
```

会弹出一个蓝色背景窗口，中间有一只海龟。

打开**第三个终端**，启动键盘控制：

```bash
rosrun turtlesim turtle_teleop_key
```

保持该终端为焦点，按方向键，海龟应随之移动。至此安装验证通过。全部验证完后在各终端按 `Ctrl+C` 退出。

## 常见问题排查

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| `apt update` 报 GPG 错误 `NO_PUBKEY F42ED6FBAB17C654` | 密钥未添加成功 | 重做第 3 步，用 keyserver 方式添加 |
| `apt update` 卡住或极慢 | 使用了官方源且网络受限 | 换清华源（第 2 步），系统源也可换 `mirrors.tuna.tsinghua.edu.cn` |
| `sudo rosdep init` 报 `Website may be down` | 无法访问 raw.githubusercontent.com | 改用 rosdepc（第 5 步） |
| `rosdep update` 超时 | 同上 | 改用 `rosdepc update` |
| `roscore: command not found` | 未 source 环境 | 执行 `source /opt/ros/noetic/setup.bash`，并确认已写入 `.bashrc` |
| turtlesim 窗口不弹出（SSH 场景） | 无图形环境 | 在本机桌面或带 X 转发（`ssh -X`）的会话中运行 |
| `Unable to register with master node` | roscore 未启动 | 先在独立终端运行 `roscore` |
