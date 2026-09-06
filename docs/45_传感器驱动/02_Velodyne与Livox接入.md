# 02 Velodyne 与 Livox 接入

> English version: [02_velodyne_and_livox.md](../en/45_sensor_drivers/02_velodyne_and_livox.md)

## 目标

- Velodyne VLP-16（3D 机械式）：以太网直连 Romindly Mind，跑通 `VLP16_points.launch`，验证 `/velodyne_points`。
- Livox（3D 固态，Horizon/Avia/Mid-40 等）：装 Livox-SDK 与 `livox_ros_driver`，理解 `xfer_format`，输出 FAST-LIO 所需的 CustomMsg。
- 明确两者输出话题与 [25 章 3D SLAM](../25_3D激光SLAM/README.md) 各方案输入的对应关系。

## Velodyne VLP-16

### 硬件连接与网络配置

VLP-16 经接口盒（Interface Box）供电 + 网线输出，出厂默认 IP **192.168.1.201**，向广播地址发 UDP 2368 端口数据。

Romindly Mind 有双 2.5G 网口，典型接法：**网口 1 直连雷达**（静态 IP），**网口 2 接外网/路由器**，互不影响：

```bash
ip link                      # 确认网口名，如 enp1s0（雷达）/ enp2s0（外网）
# 临时配置（重启失效，netplan 持久化见 04 节）
sudo ip addr add 192.168.1.100/24 dev enp1s0
sudo ip link set enp1s0 up

ping 192.168.1.201           # 通了说明物理链路正常
sudo tcpdump -i enp1s0 udp port 2368 -c 5   # 能抓到包说明雷达在出数据
```

注意静态 IP 取 192.168.1.x（x ≠ 201），且不要与网口 2 所在网段重叠。浏览器打开 `http://192.168.1.201` 可进雷达配置页（改转速、IP 等）。

### 驱动安装与启动

```bash
sudo apt install ros-noetic-velodyne          # 二进制方式
# 或源码方式（本套件 fork，melodic-devel 分支，Noetic 可用）：
cd ~/catkin_ws/src
git clone -b melodic-devel https://github.com/Romindly-Dev/velodyne.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

启动与验证：

```bash
roslaunch velodyne_pointcloud VLP16_points.launch

rostopic list | grep velodyne
# /velodyne_packets   原始 UDP 包
# /velodyne_points    PointCloud2 点云
# /scan               laserscan 节点从点云抽取的单线 LaserScan

rostopic hz /velodyne_points     # 转速 600 rpm 时约 10 Hz
rviz    # Fixed Frame 设为 velodyne，添加 PointCloud2 订阅 /velodyne_points
```

### 关键参数（VLP16_points.launch 的 arg，命令行可覆盖）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `device_ip` | 空 | 雷达 IP 过滤；多网卡/多雷达时建议显式设 `192.168.1.201` |
| `port` | 2368 | 数据 UDP 端口 |
| `frame_id` | `velodyne` | 点云坐标系名，接 SLAM 时的外参 TF 以此为准 |
| `rpm` | 600.0 | 电机转速（600 rpm = 10 Hz），需与雷达网页配置一致 |
| `min_range` / `max_range` | 0.4 / 130.0 | 距离裁剪（米），室内可把 max_range 降到 30 减少噪点 |
| `cut_angle` | -0.01 | ≥0 时按固定角度切帧；负值按包数切帧 |
| `organize_cloud` | false | 输出有组织点云（某些算法需要） |

示例：`roslaunch velodyne_pointcloud VLP16_points.launch device_ip:=192.168.1.201 max_range:=50.0`

## Livox（Horizon / Avia / Mid-40）

### Livox-SDK 安装

`livox_ros_driver` 依赖 Livox-SDK（C++ 库），安装步骤在 [25 章 3D 激光 SLAM](../25_3D激光SLAM/README.md) 的 FAST-LIO 小节已出现过，这里回顾：

```bash
sudo apt install cmake build-essential
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK/build && cmake .. && make -j$(nproc)
sudo make install
```

> Mid-360 属于新一代设备，使用 Livox-SDK2 + `livox_ros_driver2`，不在本仓库驱动支持范围内，本节以 Horizon/Avia/Mid-40 为例。

### 驱动编译与网络

```bash
cd ~/catkin_ws/src
git clone https://github.com/Romindly-Dev/livox_ros_driver.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

Livox 雷达同样走以太网（默认自动获取 65 段地址，主机网口可设 192.168.1.50/24 直连），驱动**按广播码（broadcast code，机身贴纸上的 15 位序列号）识别设备**：把它填进 `livox_ros_driver/config/livox_lidar_config.json` 的 `broadcast_code` 字段并置 `enable: true`，或作为 launch 的 `bd_list` 参数传入。

### 启动与 xfer_format

```bash
roslaunch livox_ros_driver livox_lidar_msg.launch      # CustomMsg 格式，FAST-LIO 用
# 或
roslaunch livox_ros_driver livox_lidar.launch          # PointCloud2 格式，RViz 直接可视化

rostopic hz /livox/lidar
rostopic echo /livox/imu -n 1      # 内置 IMU（Horizon/Avia）
```

`xfer_format` 决定 `/livox/lidar` 的消息类型（README 与 launch 均可核对）：

| xfer_format | 消息类型 | 用途 |
|-------------|----------|------|
| 0 | PointCloud2（PointXYZRTL，Livox 扩展字段） | RViz 可视化、通用处理（`livox_lidar.launch` 默认） |
| 1 | `livox_ros_driver/CustomMsg`（含每点相对时间戳） | **FAST-LIO/FAST-LIO2 必须**（`livox_lidar_msg.launch` 默认） |
| 2 | PointCloud2（pcl::PointXYZI） | 兼容标准 PCL 流程 |

其他常用 launch 参数：`publish_freq`（点云发布频率，5/10/20/50 Hz）、`multi_topic`（多雷达各发独立话题）、`msg_frame_id`（默认 `livox_frame`）。FAST-LIO 依赖 CustomMsg 中的逐点时间做运动畸变补偿，所以**跑 FAST-LIO 一律用 `livox_lidar_msg.launch`**。

## 对接 25 章 3D SLAM：话题对应表

以本工作空间各算法默认配置为准：

| SLAM 方案 | 期望点云话题 | 期望 IMU 话题 | 本节驱动输出 | 对接方式 |
|-----------|--------------|----------------|--------------|----------|
| A-LOAM | `/velodyne_points` | 不用 IMU | Velodyne `/velodyne_points` | 直接可用 |
| LIO-SAM | `points_raw`（params.yaml 的 `pointCloudTopic`） | `imu_raw` | Velodyne `/velodyne_points` + 外置 IMU | 改 params.yaml 话题名，或 launch 里 remap |
| FAST-LIO2（velodyne.yaml） | `/velodyne_points` | `/imu/data` | Velodyne + 外置 IMU | 点云直接可用，IMU 话题按实际驱动改 yaml |
| FAST-LIO2（avia/horizon.yaml） | `/livox/lidar`（CustomMsg） | `/livox/imu` | `livox_lidar_msg.launch` | 直接可用，选对应型号 yaml |

通用注意事项：

- LIO-SAM / FAST-LIO 对**雷达-IMU 外参与时间同步**敏感，配置细节见 25 章对应小节，本节只保证驱动侧话题正确。
- 点云 `frame_id`（`velodyne` / `livox_frame`）需与 SLAM 配置及 TF 树一致。
- 网口静态 IP 的持久化（netplan）与开机自启见 [04 udev 规则与设备管理](04_udev规则与设备管理.md)。

## 常见问题

**1. `ping` 不通雷达**：网口没起来（`ip link set ... up`）、静态 IP 网段不对、或接口盒未上电（VLP-16 需 12V 适配器）。

**2. 话题有但 `rostopic hz` 无输出**：多网卡时 UDP 包进了另一张网卡——显式指定 `device_ip`，并确认防火墙未拦 2368 端口（`sudo ufw status`）。

**3. Livox 驱动启动后一直 `wait for lidar`**：broadcast_code 填错或 `enable` 为 false；核对机身贴纸并检查 json。

**4. FAST-LIO 报点云消息类型不匹配**：用了 `livox_lidar.launch`（PointCloud2）而非 `livox_lidar_msg.launch`（CustomMsg）。

---

上一篇：[01 RPLIDAR 接入](01_RPLIDAR接入.md) | 下一篇：[03 RealSense 深度相机接入](03_RealSense深度相机接入.md)
