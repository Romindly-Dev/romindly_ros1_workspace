# 01 RPLIDAR 接入

> English version: [01_rplidar.md](../en/45_sensor_drivers/01_rplidar.md)

## 目标

- 把思岚 RPLIDAR（A1/A2/A3/S/C 系列）通过 USB 串口接到 Romindly Mind，编译 `rplidar_ros` 并发布 `/scan`。
- 用 `rostopic hz` 与 RViz 验证数据，最后接入 20 章的 gmapping / slam_toolbox 建图。

## 硬件连接

1. RPLIDAR 通过随机附带的 USB 转接板（CP210x 芯片）连接到 Romindly Mind 的 Type-C 口（可经 Type-C 转 USB-A 集线器）。
2. A 系列由 USB 直接供电即可；S 系列功耗更高，建议接**有独立供电的 USB HUB**，避免电压不足导致电机转速不稳。
3. 插入后确认设备节点出现：

```bash
ls -l /dev/ttyUSB*
dmesg | grep -i cp210x        # 应看到 "cp210x converter now attached to ttyUSB0"
```

### 串口权限

普通用户默认无权读写 `/dev/ttyUSB0`。推荐做法是加入 `dialout` 组（一次配置永久生效）：

```bash
sudo usermod -aG dialout $USER
# 注销重新登录后生效，用 groups 命令确认
```

临时方案（重启/重插后失效，仅应急用）：

```bash
sudo chmod 666 /dev/ttyUSB0
```

> 多串口设备共存时 `/dev/ttyUSB0` 会漂移，正式部署请按 [04 udev 规则](04_udev规则与设备管理.md) 固定为 `/dev/rplidar`。

## 驱动安装

使用组织 fork 的 Slamtec 官方驱动（master 分支）：

```bash
cd ~/catkin_ws/src
git clone https://github.com/Romindly-Dev/rplidar_ros.git
cd ~/catkin_ws
catkin_make            # 或 catkin build
source devel/setup.bash
```

## 启动与验证

驱动按型号提供独立 launch 文件（`rplidar_ros/launch/`），核心节点均为 `rplidarNode`，区别主要在波特率：

```bash
# 按你的型号选择其一
roslaunch rplidar_ros rplidar_a1.launch      # A1 / A2M8
roslaunch rplidar_ros rplidar_a3.launch      # A3
roslaunch rplidar_ros rplidar_s2.launch      # S2
```

正常启动日志会打印 SDK 版本、序列号和 `current scan mode`。随后验证：

```bash
rostopic hz /scan
# A1 约 5.5~10 Hz，S/C 系列约 10 Hz，rate 稳定即正常

rostopic echo /scan -n 1 | head -20
# 关注 header.frame_id: "laser"，ranges 数组有有效距离值
```

RViz 查看（驱动自带 view_ 系列 launch，含 rviz 配置）：

```bash
roslaunch rplidar_ros view_rplidar_a1.launch    # 型号对应替换
```

或手动启动 `rviz`：Fixed Frame 设为 `laser`，添加 LaserScan 显示项订阅 `/scan`。

## 参数表（rplidarNode）

以下参数取自本仓库各 launch 文件，讲解以实际文件为准：

| 参数 | 类型 | 说明 | 默认/典型值 |
|------|------|------|-------------|
| `serial_port` | string | 串口设备名 | `/dev/ttyUSB0`（部署时改 `/dev/rplidar`） |
| `serial_baudrate` | int | 波特率，**按型号不同**，见下表 | 115200 |
| `frame_id` | string | 扫描数据坐标系名 | `laser` |
| `inverted` | bool | 雷达是否倒装 | `false` |
| `angle_compensate` | bool | 角度补偿，使每帧点数均匀 | `true` |
| `scan_mode` | string | 扫描模式（A3 launch 用 `Sensitivity`，C1 用 `Standard`） | 空=固件默认 |
| `scan_frequency` | double | 扫描频率 Hz（S2/C1 等新型号 launch 提供） | `10.0` |

各型号波特率（以仓库 launch 文件为准）：

| 型号 | launch 文件 | serial_baudrate |
|------|-------------|-----------------|
| A1 / A2M7 / A2M8 | `rplidar_a1.launch` 等 | 115200 |
| A2M12 / A3 / S1 | `rplidar_a2m12.launch` / `rplidar_a3.launch` / `rplidar_s1.launch` | 256000 |
| C1 | `rplidar_c1.launch` | 460800 |
| S2 / S3 / S2E | `rplidar_s2.launch` 等 | 1000000 |

> 波特率不匹配的典型现象是启动报 `Error, operation time out`——先核对型号与 launch 是否对应。

## 对接 20 章建图

gmapping / slam_toolbox 需要 `/scan` 加上 `base_link → laser` 的 TF。最小组合 launch（对照 [20 章 gmapping 实战](../20_2D建图/02_gmapping实战.md)）：

```xml
<launch>
  <!-- 雷达驱动 -->
  <include file="$(find rplidar_ros)/launch/rplidar_a1.launch"/>

  <!-- base_link -> laser 静态 TF，平移按实机安装位置修改 -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="laser_tf"
        args="0.10 0 0.18 0 0 0 base_link laser"/>

  <!-- gmapping（还需底盘提供 odom->base_link TF） -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <remap from="scan" to="/scan"/>
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="odom"/>
  </node>
</launch>
```

要点：

- `frame_id`（默认 `laser`）必须与静态 TF 的子坐标系一致，否则 RViz 报 TF 缺失、gmapping 无输出。
- gmapping 还依赖里程计 TF（`odom → base_link`），纯雷达无底盘时可先用 [20 章 hector_slam](../20_2D建图/03_hector_slam实战.md) 验证。
- [robot_bringup 的 sensors.launch](https://github.com/Romindly-Dev/romindly_robot_bringup) 已内置本节的驱动 + 静态 TF 组合，实机部署直接 `roslaunch robot_bringup sensors.launch use_rplidar:=true`。
- slam_toolbox 对接同理，见 [20 章 slam_toolbox 实战](../20_2D建图/04_slam_toolbox实战.md)。

## 常见问题

**1. `Error, operation time out. RESULT_OPERATION_TIMEOUT`**
- 波特率与型号不匹配（最常见），对照上面波特率表换 launch；
- 串口被其他进程占用（`sudo lsof /dev/ttyUSB0` 排查）；
- USB 线过长或质量差，换短线直连。

**2. 打开串口失败 / Permission denied**
- 未加入 `dialout` 组或未注销重登；应急 `sudo chmod 666 /dev/ttyUSB0`。

**3. 电机不转或转速忽快忽慢、`/scan` 频率不稳**
- USB 供电不足，S 系列尤其明显；换带独立供电的 HUB 或主机原生 USB3 口。

**4. 设备名不是 ttyUSB0**
- 多个串口设备并存时编号随插拔顺序漂移；用 `serial_port:=/dev/ttyUSB1` 临时指定，长期方案见 [04 udev 规则](04_udev规则与设备管理.md)。

**5. RViz 中点云方向反了**
- 雷达倒装时设 `inverted:=true`；安装朝向偏差用静态 TF 的 yaw 修正。

---

下一篇：[02 Velodyne 与 Livox 接入](02_Velodyne与Livox接入.md)
