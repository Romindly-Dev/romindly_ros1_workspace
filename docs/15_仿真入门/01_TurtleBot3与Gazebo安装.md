# 01 TurtleBot3 与 Gazebo 安装

> English version: [01_turtlebot3_gazebo_install.md](../en/15_simulation/01_turtlebot3_gazebo_install.md)

## 目标

- 确认 Gazebo 11 已随 desktop-full 安装并能正常启动
- 用 apt 装好 TurtleBot3 三件套（仿真、消息、遥控）
- 理解 `TURTLEBOT3_MODEL` 环境变量的作用并写入 `~/.bashrc`
- 跑通第一个仿真世界 `turtlebot3_world.launch`

## 原理简介

**Gazebo** 是 ROS1 生态的标准物理仿真器：它模拟刚体动力学、传感器（激光、IMU、相机）和执行器，并通过 `gazebo_ros` 桥接包把仿真数据以 ROS 话题/TF 的形式发出来。对上层算法（建图、导航）而言，**仿真机器人和实体机器人发布的话题接口完全一致**——这就是"先仿真后实机"能成立的原因。

**TurtleBot3**（下称 TB3）是 ROBOTIS 出品的开源教学机器人，是 ROS 社区事实上的标准练习平台，官方提供了完整的 Gazebo 模型、建图和导航配置。它由三个包群组成：

| 仓库 | 内容 |
| --- | --- |
| `turtlebot3` | 机器人描述（URDF）、bringup、teleop、navigation 配置 |
| `turtlebot3_msgs` | TB3 专用消息定义 |
| `turtlebot3_simulations` | Gazebo 世界、仿真模型与插件配置 |

组织已 fork 这三个仓库（noetic 分支）并纳入 `vcs import` 清单，但**教学阶段推荐直接 apt 安装**——省去编译，且版本经过 ROBOTIS 官方发布验证。

## 分步操作

### 第 1 步：验证 Gazebo

00 章安装的 `ros-noetic-desktop-full` 已自带 Gazebo 11，先确认：

```bash
gazebo --version
```

预期输出：

```
Gazebo multi-robot simulator, version 11.15.1
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org
```

版本号 11.x 即可（小版本随更新有差异）。如果提示 `command not found`，说明装的不是 desktop-full，补装：`sudo apt install ros-noetic-desktop-full`。

可以裸启动看一眼（一个空世界 + 地面 + 光源，看完关掉）：

```bash
gazebo
```

### 第 2 步：安装 TurtleBot3（apt，推荐）

```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-teleop
```

`ros-noetic-turtlebot3` 是元包，会自动带上 `turtlebot3-msgs`、`turtlebot3-description` 等依赖。验证：

```bash
rospack find turtlebot3_gazebo
```

预期输出：

```
/opt/ros/noetic/share/turtlebot3_gazebo
```

> **备选：源码安装。** 如果后续需要改 TB3 的模型或插件参数，可用源码方式：00 章的 `vcs import` 清单已包含组织 fork 的 `turtlebot3` / `turtlebot3_msgs` / `turtlebot3_simulations`（noetic 分支），拉取后在 `~/ws_romindly` 下 `catkin_make` 即可。源码包会覆盖同名 apt 包（工作空间 overlay 优先）。教程默认按 apt 方式讲。

### 第 3 步：设置 TURTLEBOT3_MODEL

TB3 的 launch 文件通过环境变量 `TURTLEBOT3_MODEL` 决定加载哪款机器人的 URDF，**不设置就直接报错退出**。本套教程统一用 `burger`：

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
echo $TURTLEBOT3_MODEL   # 应输出 burger
```

三款型号的区别：

| 型号 | 传感器 | 最大速度（线/角） | 特点 |
| --- | --- | --- | --- |
| `burger` | 360° 激光雷达（LDS） | 0.22 m/s / 2.84 rad/s | 最小最轻，2D 建图导航教学首选 |
| `waffle` | 激光雷达 + RealSense 深度相机 | 0.26 m/s / 1.82 rad/s | 底盘更宽，带视觉 |
| `waffle_pi` | 激光雷达 + 树莓派相机 | 0.26 m/s / 1.82 rad/s | 同 waffle 底盘，相机不同 |

本章到 40 章的实验用 `burger` 就够了（建图导航只需要激光）；涉及相机的实验会单独提示切换 `waffle_pi`。

### 第 4 步：首次启动仿真世界

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

预期：Gazebo 窗口打开，出现一个六边形围栏、内部立着几根圆柱的场地，中央一台黑色小车（burger）。终端可见：

```
[ INFO] [...]: Finished loading Gazebo ROS API Plugin.
[ INFO] [...]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [...]: Physics dynamic reconfigure ready.
```

> **首次启动会慢（可能卡 1~3 分钟黑屏）**：Gazebo 第一次运行会尝试连接在线模型库下载 `sun`、`ground_plane` 等基础模型并建立本地缓存（`~/.gazebo/models/`）。耐心等待即可，第二次启动就快了。如果网络不通导致长时间无响应，见下方常见问题。

确认 ROS 侧接口正常（另开终端）：

```bash
rostopic list
```

应能看到 `/scan`、`/odom`、`/cmd_vel`、`/imu`、`/clock` 等话题——下一篇逐个观察它们。

关闭仿真：在 roslaunch 终端按 `Ctrl+C`，等它自己退干净（Gazebo 关闭较慢，别连按）。

## 讲解

- `turtlebot3_world.launch` 做了三件事：启动 Gazebo 并加载 `turtlebot3_world` 世界文件；按 `TURTLEBOT3_MODEL` 读取 URDF 并 spawn 机器人到仿真中；启动 `robot_state_publisher` 发布机器人内部 TF（03 篇细讲）。
- Gazebo 实际是两个进程：`gzserver`（物理计算，无界面）+ `gzclient`（3D 显示窗口）。异常退出时 gzserver 常残留在后台，导致下次启动报错——这是 TB3 仿真最高频的问题，见下面。
- 其他可玩的世界：`turtlebot3_empty_world.launch`（空地）、`turtlebot3_house.launch`（多房间住宅，模型多、首次加载更慢，适合 40 章导航实验）。

## 动手练习

1. 关闭当前仿真，启动 `turtlebot3_house.launch`，在 Gazebo 中用鼠标（左键平移、滚轮缩放、Shift+左键旋转视角）浏览整个房子。
2. 临时把模型换成 waffle：`TURTLEBOT3_MODEL=waffle roslaunch turtlebot3_gazebo turtlebot3_world.launch`，对比小车外观差异（这种前缀写法只对本条命令生效，不影响 bashrc 里的设置）。
3. 故意不设环境变量启动一次：`env -u TURTLEBOT3_MODEL roslaunch turtlebot3_gazebo turtlebot3_world.launch`，观察报错信息，记住这个错误的样子。

## 常见问题

**Q1：启动报错 `[Err] [RTShaderSystem.cc] ... unable to find ...` 或直接 `gzserver` 崩溃，提示地址被占用。**
上一次仿真没退干净，gzserver 残留。清理后重启：

```bash
killall -9 gzserver gzclient
```

养成习惯：每次 `Ctrl+C` 后等终端完全回到提示符再启动下一次。

**Q2：虚拟机里 Gazebo 黑屏/花屏/秒崩（常见报错含 `OGRE EXCEPTION` 或 `VBO` 字样）。**
虚拟机 3D 加速与 Gazebo 的 OpenGL 需求冲突。两个办法：

```bash
# 办法一：强制软件渲染（慢但稳）
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

办法二：在虚拟机设置中关闭"3D 图形加速"。本套件的边缘计算单元是物理机 + 核显，一般不会遇到；此问题多见于学员自带的 VMware/VirtualBox 环境。

**Q3：报错 `TURTLEBOT3_MODEL is not set`。**
环境变量没生效。新开的终端要重新 `source ~/.bashrc`（或确认第 3 步的 echo 确实写进去了：`grep TURTLEBOT3 ~/.bashrc`）。

**Q4：世界加载出来但场地/家具缺失，终端刷 `waiting for model` 或模型下载超时。**
Gazebo 在找不到本地模型时会去在线模型库下载，网络不通就一直等。TB3 自带的模型其实都在本地包里，确认模型路径已包含它们：

```bash
echo $GAZEBO_MODEL_PATH
```

正常应包含 `/opt/ros/noetic/share/turtlebot3_gazebo/models`（由 `source /opt/ros/noetic/setup.bash` 时的包环境钩子自动追加）。如果为空，手动补上：

```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

**Q5：仿真跑起来很卡，Gazebo 底部状态栏 Real Time Factor 远小于 1.0。**
机器性能不足或软件渲染导致。教学场景下 RTF ≥ 0.8 即可接受；关掉不必要的 Gazebo 窗口特效（左侧面板 → 阴影），或换 `turtlebot3_empty_world.launch` 这类轻量世界。

下一篇：[02 键盘遥控与话题观察](02_键盘遥控与话题观察.md)
