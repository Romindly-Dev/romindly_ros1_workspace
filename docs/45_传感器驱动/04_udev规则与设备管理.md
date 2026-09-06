# 04 udev 规则与设备管理

## 目标

- 理解串口设备名漂移的原因，用 udev 规则为 RPLIDAR、IMU、底盘串口生成固定软链 `/dev/rplidar`、`/dev/imu`、`/dev/base_serial`。
- 用 netplan 为 Velodyne/Livox 场景的网口配置持久静态 IP。
- 为 [50 章部署运维](../50_部署运维/README.md) 的开机自启做好"设备就绪"前置检查。

## 为什么设备名会漂移

Linux 按**枚举顺序**给 USB 串口分配 `/dev/ttyUSB0`、`/dev/ttyUSB1`……哪个设备先被内核发现就拿到小编号。插拔顺序、上电时序、HUB 端口都会改变枚举顺序：今天雷达是 `ttyUSB0`、IMU 是 `ttyUSB1`，重启后可能对调。launch 文件里写死 `ttyUSB0` 的结果就是"雷达驱动去打开 IMU 串口"，随机失败且难排查。

解决办法：udev 在设备插入时按 **VID/PID/序列号** 等稳定属性匹配，创建固定名字的软链接，launch 全部改用软链名。[robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) 的 `sensors.launch` 与 `config/base.yaml` 用的正是 `/dev/rplidar`、`/dev/base_serial`。

## 第一步：查设备属性

插入设备后，用 `udevadm info` 查它的身份信息：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial" | head -6
```

典型输出（RPLIDAR 的 CP210x 转接板）：

```
ATTRS{idVendor}=="10c4"
ATTRS{idProduct}=="ea60"
ATTRS{serial}=="0001"
```

常见芯片：CP210x（`10c4:ea60`，思岚雷达常用）、CH340（`1a86:7523`，很多国产 IMU/底盘板）、FTDI（`0403:6001`）。逐个插入设备记录三元组，**同型号芯片必须靠 `serial` 区分**。

## 第二步：编写规则文件

创建 `/etc/udev/rules.d/99-robot-devices.rules`（数字前缀决定顺序，99 保证在系统默认规则之后）：

```bash
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```

```
# RPLIDAR（CP210x），软链 /dev/rplidar，顺带放开权限（可代替 dialout 组方案）
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE="0666", SYMLINK+="rplidar"

# IMU（CH340），软链 /dev/imu
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE="0666", SYMLINK+="imu"

# 底盘串口（FTDI），软链 /dev/base_serial —— 对应 robot_bringup/config/base.yaml 的 port
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", \
  MODE="0666", SYMLINK+="base_serial"
```

> VID/PID 以你 `udevadm info` 的实测值为准，上面只是常见芯片示例。

### 同型号设备冲突：用 serial 区分

如果 IMU 和底盘用了同款 CH340，仅靠 VID/PID 两条规则会都匹配上。加 `ATTRS{serial}`（或没有序列号时用 `KERNELS=="1-2"` 这类物理端口路径）精确区分：

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="IMU0001", SYMLINK+="imu", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="BASE001", SYMLINK+="base_serial", MODE="0666"
```

## 第三步：加载与验证

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger            # 或重新插拔设备
ls -l /dev/rplidar /dev/imu /dev/base_serial
# lrwxrwxrwx 1 root root 7 ... /dev/rplidar -> ttyUSB0
```

之后 launch 一律用软链名，例如 [01 节](01_RPLIDAR接入.md) 的雷达：

```bash
roslaunch rplidar_ros rplidar_a1.launch serial_port:=/dev/rplidar
# robot_bringup 的 sensors.launch 已默认使用 /dev/rplidar
```

## 网口设备：netplan 静态 IP（Velodyne 场景）

[02 节](02_Velodyne与Livox接入.md) 里 `ip addr add` 是临时的，重启即失效。Ubuntu 20.04 用 netplan 持久化。编辑 `/etc/netplan/01-lidar.yaml`（网口名按 `ip link` 实际输出）：

```yaml
network:
  version: 2
  ethernets:
    enp1s0:                 # 网口 1：直连 Velodyne（默认 192.168.1.201）
      addresses: [192.168.1.100/24]
      dhcp4: false
    enp2s0:                 # 网口 2：外网，保持 DHCP
      dhcp4: true
```

```bash
sudo netplan apply
ping 192.168.1.201
```

注意：雷达口**不要配网关**，否则可能抢默认路由导致外网口断网；两个网口网段不能重叠。

## 开机自启前的设备就绪检查

[50 章](../50_部署运维/README.md) 用 `ros-robot.service` 开机自启。系统服务启动时 USB 枚举可能还没完成，直接启动 launch 会因设备不存在而失败。两种做法：

**方案 A：udev TAG + systemd 设备单元（推荐）。** 在规则里给设备打 systemd 标签：

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE="0666", SYMLINK+="rplidar", TAG+="systemd", ENV{SYSTEMD_ALIAS}="/dev/rplidar"
```

然后在 `ros-robot.service` 的 `[Unit]` 段追加依赖（`dev-rplidar.device` 是 systemd 对 `/dev/rplidar` 的单元名）：

```ini
[Unit]
Description=ROS1 Robot Bringup
After=network-online.target dev-rplidar.device
Wants=network-online.target
Requires=dev-rplidar.device
```

这样服务会等雷达设备就绪才启动，拔掉设备时服务也不会盲跑。

**方案 B：启动脚本内轮询。** 在 `ros_env.sh` 开头加等待逻辑，简单粗暴但通用：

```bash
for i in $(seq 1 30); do
  [ -e /dev/rplidar ] && break
  sleep 1
done
```

配合 `Restart=on-failure`（服务模板已带），即使偶发未就绪也能自愈。

## 常见问题

**1. 规则写了但软链没出现**：`udevadm control --reload-rules` 后必须**重新插拔**或 `udevadm trigger`；再用 `udevadm test $(udevadm info -q path -n /dev/ttyUSB0) 2>&1 | grep -i symlink` 看规则是否命中。

**2. 软链指向了错误的设备**：两台设备 VID/PID 相同且没加 `serial` 条件；按上文用序列号或物理端口路径区分。

**3. `ATTRS{serial}` 查不到**：廉价 CH340 芯片常无序列号，改用 `KERNELS=="1-2"`（`udevadm info -a` 里 KERNELS 值，绑定物理 USB 口，代价是设备不能换口插）。

**4. netplan apply 后外网断了**：雷达口配了 `gateway4`/`routes` 抢走默认路由，删掉即可。

---

上一篇：[03 RealSense 深度相机接入](03_RealSense深度相机接入.md) | 返回 [本章索引](README.md)
