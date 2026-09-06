# 02 · gmapping 实战

## 目标

- 在 TurtleBot3 仿真中用 gmapping 完成一次完整建图并保存地图
- 理解 gmapping（RBPF 粒子滤波）核心参数的物理含义与调参方向
- 能诊断地图质量差的典型原因

## 原理简介

gmapping 是 RBPF（Rao-Blackwellized Particle Filter）SLAM 的经典实现，ROS 包为 `slam_gmapping`（封装 `openslam_gmapping` 算法库）。每个粒子携带一条完整轨迹假设和一张自己的地图；每当机器人移动超过 `linearUpdate`/`angularUpdate` 阈值，就执行一轮"里程计传播 → 激光扫描匹配修正 → 打分 → 按需重采样"。它**没有回环检测**，依赖里程计与扫描匹配抑制漂移，因此对里程计质量敏感。

源码阅读：`Romindly-Dev/slam_gmapping`（上游 ros-perception），参数默认值见 `gmapping/src/slam_gmapping.cpp` 的 `SlamGMapping::init()`。

## 分步操作

以下每步开一个新终端，均先 `export TURTLEBOT3_MODEL=burger`。

### 1. 启动仿真

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Gazebo 打开六边形世界，终端出现 `odom received!` 表示底盘就绪。

### 2. 启动 gmapping（含 RViz）

TB3 快捷方式：

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

预期输出（节选）：

```text
[ INFO] [...]: Laser is mounted upwards.
Registering First Scan
```

RViz 自动打开并显示以机器人为中心逐渐展开的地图。

通用方式（实机 / 非 TB3 底盘时使用，前提是 `/scan` 与 `odom→base_link` TF 已就绪）：

```bash
rosrun gmapping slam_gmapping scan:=scan _base_frame:=base_footprint _odom_frame:=odom \
    _map_update_interval:=2.0 _maxUrange:=3.0 _particles:=30
```

实机项目建议固化成 launch 文件（参数一目了然、便于版本管理）：

```xml
<launch>
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <remap from="scan" to="/scan"/>
    <param name="base_frame" value="base_link"/>   <!-- 按自己底盘修改 -->
    <param name="odom_frame" value="odom"/>
    <param name="map_frame"  value="map"/>
    <param name="particles" value="50"/>
    <param name="linearUpdate" value="0.3"/>
    <param name="angularUpdate" value="0.25"/>
    <param name="map_update_interval" value="2.0"/>
    <param name="maxUrange" value="10.0"/>         <!-- 雷达可靠量程的 80% -->
    <param name="minimumScore" value="50"/>
    <param name="delta" value="0.05"/>
    <param name="srr" value="0.1"/> <param name="srt" value="0.2"/>
    <param name="str" value="0.1"/> <param name="stt" value="0.2"/>
  </node>
</launch>
```

RViz 手动配置：Fixed Frame 设为 `map`，添加 `Map`（话题 `/map`）与 `LaserScan`（话题 `/scan`）显示项。

启动后自检（新终端）：

```bash
rosnode info /slam_gmapping        # 确认订阅 /scan、/tf，发布 /map
rosrun tf tf_echo map odom          # gmapping 已开始发布修正量
rostopic hz /map                    # 约每 map_update_interval 秒一帧
```

### 3. 遥控建图

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

操作要领：线速度 ≤ 0.15 m/s，**转弯要慢**（角速度 ≤ 0.5 rad/s）；沿墙走一整圈，最后回到起点附近观察闭合处是否错位。

### 4. 保存地图

建图满意后**不要关闭任何节点**，新终端执行：

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/tb3_world_gmapping
```

预期输出：

```text
[ INFO] [...]: Waiting for the map
[ INFO] [...]: Received a 384 X 384 map @ 0.050 m/pix
[ INFO] [...]: Writing map occupancy data to /home/iot/maps/tb3_world_gmapping.pgm
[ INFO] [...]: Writing map occupancy data to /home/iot/maps/tb3_world_gmapping.yaml
[ INFO] [...]: Done
```

`map_saver` 只是订阅一次 `/map` 落盘，保存完即可 Ctrl+C 关闭全部节点。验证：

```bash
rosrun map_server map_server ~/maps/tb3_world_gmapping.yaml   # 重新发布 /map，用 RViz 查看
```

`map.yaml` 字段含义见 [01 篇](./01_建图原理与方案选型.md#占据栅格地图)。注意 `image` 字段是相对路径，移动地图文件时 pgm 与 yaml 要一起移动。

## 参数详解

默认值核对自 `slam_gmapping.cpp`（Noetic 分支）。以下按重要程度排列：

| 参数 | 默认值 | 作用 | 调参建议 |
| --- | --- | --- | --- |
| `particles` | 30 | 粒子数，每个粒子一张地图 | 小场景 30 足够；大场景/里程计差提到 50–80；CPU 与内存随之线性增长 |
| `linearUpdate` | 1.0 (m) | 平移超过该值才处理一帧激光 | 常改为 0.2–0.5，更新更密、地图更细致，代价是计算量增大 |
| `angularUpdate` | 0.5 (rad) | 旋转超过该值才处理一帧激光 | 常改为 0.2–0.25，缓解转弯时地图错位 |
| `temporalUpdate` | -1.0 (s) | 静止超时也强制更新；负值禁用 | 一般保持禁用；动态环境可设 3–5 s |
| `map_update_interval` | 5.0 (s) | `/map` 话题发布周期 | 只影响可视化刷新，不影响精度；调 2.0 观感更好 |
| `maxUrange` | 等于 maxRange | 用于**建图**的激光最大距离 | 设为雷达可靠量程的 80%（TB3 LDS 设 3.0–3.5）；过大引入远距噪点 |
| `maxRange` | scan.range_max − 0.01 | 用于**清除空闲区**的最大距离 | 一般设雷达标称量程；保持 maxUrange < maxRange |
| `srr` | 0.1 | 里程计噪声模型：平移引起的平移误差 | 里程计打滑严重（地毯/差速轮）适当调大到 0.2 |
| `srt` | 0.2 | 旋转引起的平移误差 | 同上；调大 = 更不信任里程计，更依赖扫描匹配 |
| `str` | 0.1 | 平移引起的旋转误差 | 通常不动 |
| `stt` | 0.2 | 旋转引起的旋转误差 | 转弯后地图扭曲时适当调大 |
| `minimumScore` | 0 | 扫描匹配得分低于此值则退回里程计 | 大空旷区（激光打不到墙）设 50–200 防止匹配乱跳 |
| `delta` | 0.05 (m) | 地图分辨率 | 通用 0.05；精细场景 0.025（开销约 4 倍） |
| `xmin/ymin/xmax/ymax` | ±100.0 (m) | 初始地图范围 | 已知场景大小可缩小以省内存（gmapping 会自动扩图） |
| `iterations` | 5 | 每帧扫描匹配优化迭代次数 | 一般不动 |
| `sigma` / `lsigma` | 0.05 / 0.075 | 匹配端点评分的高斯方差 / 似然计算方差 | 一般不动 |
| `kernelSize` | 1 | 匹配时对应点搜索窗口（格） | 里程计很差时可设 2–3，代价大 |
| `lskip` | 0 | 每帧跳过的激光束数（0=全用） | CPU 紧张时设 1–2 |
| `resampleThreshold` | 0.5 | Neff/N 低于该比例触发重采样 | 一般不动 |
| `throttle_scans` | 1 | 每 n 帧激光处理 1 帧 | 高帧率雷达（>15 Hz）可设 2 |
| `transform_publish_period` | 0.05 (s) | `map→odom` TF 发布周期 | 一般不动 |
| `occ_thresh` | 0.25 | 栅格判占据的概率阈值 | 一般不动 |

TB3 的 `turtlebot3_slam` 包在 `config/gmapping_params.yaml` 中已覆盖了大部分关键参数（如 `maxUrange: 3.0`、`linearUpdate: 1.0`、`map_update_interval: 2.0`），实机调参时以自己的 launch/yaml 显式覆盖为准。

## 常见问题

**Q1：地图出现重影 / 墙变成双层？**
典型原因是**里程计漂移**：轮子打滑、轮径参数不准、或 `odom→base_link` TF 时间戳滞后。对策：标定轮径与轮距；调大 `srr/srt/stt` 降低对里程计的信任；降低行驶速度。

**Q2：转弯后整块地图旋转错位？**
**转弯过快**是 gmapping 最常见的翻车原因——两次更新之间旋转过大，扫描匹配落入局部极值。对策：遥控时角速度压到 0.5 rad/s 以下；`angularUpdate` 调小到 0.2；粒子数适当加大。

**Q3：报错 `Scan Matching Failed, using odometry.`？**
偶发属正常（该帧退回里程计）。持续出现说明环境特征太少（长走廊、空旷大厅）或 `maxUrange` 太小导致激光打不到墙——gmapping 在此类场景先天不足，考虑换 slam_toolbox / cartographer。

**Q4：`map_saver` 一直 `Waiting for the map`？**
`/map` 还没发布过（刚启动，未到 `map_update_interval`）或话题名不对。`rostopic list | grep map` 确认；命名空间下的话题用 `map_saver map:=/xxx/map`。

**Q5：回到起点发现闭合处错开几十厘米？**
gmapping 无回环检测，绕大圈的累积误差无法消除。小错位可接受（导航靠 AMCL 兜底）；大错位请减速重扫，或改用带回环的 [slam_toolbox](./04_slam_toolbox实战.md)。
