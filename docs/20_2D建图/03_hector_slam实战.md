# 03 · hector_slam 实战

> English version: [03_hector_slam.md](../en/20_2d_slam/03_hector_slam.md)

## 目标

- 理解 hector_slam 无里程计建图的原理与适用边界
- 掌握 `hector_mapping` 的启动方法与 TF 配置要点（这是 hector 上手最容易踩的坑）
- 能应对快速旋转导致的定位丢失

## 原理简介

hector_slam（TU Darmstadt 开发，包名 `hector_mapping`）是纯扫描匹配方案：每帧激光通过**高斯牛顿法**直接对齐到当前占据栅格地图，配合**多分辨率地图金字塔**（粗图先对齐、细图再精修）避免局部极值。它完全不使用里程计，因此：

- **适用场景**：手持雷达快速扫图、无编码器的低成本底盘、履带车等里程计极不可靠的平台、灾难救援机器人（hector 的出身）。
- **代价**：定位完全依赖相邻两帧激光的重叠度。雷达帧率越高、运动越慢，匹配越稳；**快速旋转**时帧间重叠骤减，极易丢失定位且无里程计兜底。无回环检测，长走廊等低特征场景会漂移。

源码阅读：`Romindly-Dev/hector_slam`（上游 tu-darmstadt-ros-pkg），参数默认值见 `hector_mapping/src/HectorMappingRos.cpp`，参考 launch 为 `hector_mapping/launch/mapping_default.launch`。

## TF 配置要点（先看这里）

hector 输出的 TF 由两个参数决定，实机接线前必须想清楚：

- `pub_map_odom_transform`（默认 `true`）：发布 `map → odom` TF。此时要求 TF 树中**已存在** `odom → base_link`（谁发布都行）。
- 若底盘**完全没有里程计**：把 `odom_frame` 直接设成与 `base_frame` 相同（如都设 `base_link`），hector 就直接发布 `map → base_link`，TF 树最简单。
- `base_frame` 默认 `base_link`，`odom_frame` 默认 `odom`；`mapping_default.launch` 中被改成了 `base_footprint` / `nav`，**直接套用该 launch 到自己的机器人时必须改回自己的 frame 名**，否则 TF 对不上、地图不动。
- hector 还会发布 `map → scanmatcher_frame`（匹配结果调试用），可通过 `pub_map_scanmatch_transform` 关闭。

## 分步操作

每个终端先 `export TURTLEBOT3_MODEL=burger`。TB3 仿真自带里程计，这里演示两种模式。

### 1. 启动仿真

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### 2a. TB3 快捷启动

```bash
sudo apt install ros-noetic-hector-slam      # 首次
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector
```

RViz 打开后地图立即出现（hector 不等移动阈值，第一帧就注册）。预期日志：

```text
HectorSM map lvl 0: cellLength: 0.05 res x: 2048 res y: 2048
HectorSM map lvl 1: cellLength: 0.1 res x: 1024 res y: 1024
[ INFO] [...]: HectorSlamRos node started.
```

### 2b. 通用启动（模拟"无里程计"实机写法）

```bash
rosrun hector_mapping hector_mapping scan:=scan \
    _base_frame:=base_footprint _odom_frame:=base_footprint \
    _pub_map_odom_transform:=false \
    _map_resolution:=0.05 _map_size:=1024 \
    _map_update_distance_thresh:=0.1 _map_update_angle_thresh:=0.06
```

此时 hector 直接发布 `map → base_footprint`，完全不依赖轮式里程计——这就是手持雷达建图的写法。

**手持雷达建图完整模板**（实机，以 RPLIDAR 为例，可直接抄）：

```xml
<launch>
  <!-- 1. 雷达驱动（按实际雷达替换） -->
  <node pkg="rplidar_ros" type="rplidarNode" name="rplidar">
    <param name="frame_id" value="laser"/>
  </node>

  <!-- 2. 手持无底盘：base_link 与 laser 之间给一个固定 TF -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser"
        args="0 0 0 0 0 0 base_link laser 50"/>

  <!-- 3. hector：无里程计模式，直接发布 map->base_link -->
  <node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="base_link"/>
    <param name="pub_map_odom_transform" value="false"/>
    <param name="map_resolution" value="0.05"/>
    <param name="map_size" value="2048"/>
    <param name="map_update_distance_thresh" value="0.1"/>
    <param name="map_update_angle_thresh" value="0.06"/>
    <param name="laser_max_dist" value="12.0"/>
  </node>
</launch>
```

启动后自检：

```bash
rostopic hz /scan                 # 手持建图强依赖帧率，10 Hz 以下请放慢移动
rostopic echo /poseupdate -n1     # hector 输出的当前位姿估计（带协方差）
rosrun tf tf_echo map base_link   # 移动雷达应看到位姿变化
```

### 3. 遥控与保存

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
# 建图时缓慢移动，转弯格外慢；完成后：
rosrun map_server map_saver -f ~/maps/tb3_world_hector
```

hector 也提供内置保存方式（连轨迹一起存）：`rostopic pub syscommand std_msgs/String "savegeotiff"`（需运行 hector_geotiff 节点），教学阶段用 map_saver 即可。

## 参数详解

默认值核对自 `HectorMappingRos.cpp`（括号内为 `mapping_default.launch` 的覆盖值）：

| 参数 | 默认值 | 作用 | 调参建议 |
| --- | --- | --- | --- |
| `map_resolution` | 0.025 (launch: 0.05) | 最细层地图分辨率 (m) | 导航用图 0.05 足够；0.025 匹配更准但 CPU/内存翻 4 倍 |
| `map_size` | 1024 (launch: 2048) | 地图边长（格数），实际尺寸 = size×resolution | 0.05×2048 ≈ 102 m 见方；hector **不会自动扩图**，场景大要提前给够 |
| `map_start_x` / `map_start_y` | 0.5 / 0.5 | 起点在地图中的相对位置（0~1） | 0.5 = 从地图中心开建；单方向扫图可偏置到 0.2 等 |
| `map_multi_res_levels` | 3 (launch: 2) | 多分辨率金字塔层数 | 层数多抗快速运动能力强；2–3 均可 |
| `map_update_distance_thresh` | 0.4 (m) | 平移超过该值才把当前帧写入地图 | 调小到 0.1–0.2 地图更新更细；注意这只控制**写图**，位姿估计每帧都在做 |
| `map_update_angle_thresh` | 0.9 (launch: 0.06) (rad) | 旋转写图阈值 | 建议 0.06–0.1，默认 0.9 会导致旋转时长时间不更新地图 |
| `update_factor_free` | 0.4 | 空闲栅格的概率更新因子（<0.5 削减占据概率） | 动态障碍多（行人）可降到 0.3–0.35，残影消得更快 |
| `update_factor_occupied` | 0.9 | 占据栅格的概率更新因子 | 一般不动 |
| `laser_min_dist` / `laser_max_dist` | 0.4 / 30.0 (m) | 参与匹配的激光距离范围 | 按雷达实际可靠量程设置；TB3 LDS 设 max 3.5 |
| `laser_z_min_value` / `laser_z_max_value` | -1.0 / 1.0 (m) | 相对雷达高度的点过滤（斜装/翻滚时用） | 水平安装保持默认 |
| `pub_map_odom_transform` | true | 是否发布 map→odom | 见上文 TF 要点 |
| `base_frame` / `map_frame` / `odom_frame` | base_link / map / odom | 坐标系名称 | 按实际 TF 树填写；无里程计时 odom_frame = base_frame |
| `use_tf_scan_transformation` | true | 用 TF 把激光点变换到 base_frame | 保持 true，并保证 `base→laser` TF 正确 |
| `scan_topic` | scan | 激光话题名 | — |
| `scan_subscriber_queue_size` | 5 | 激光订阅队列 | 离线回灌 rosbag 时可加大 |
| `map_pub_period` | 2.0 (s) | `/map` 发布周期 | 只影响可视化 |
| `pub_odometry` | false | 以 nav_msgs/Odometry 发布匹配位姿（话题 `scanmatch_odom`） | 需要把 hector 当"激光里程计"喂给其他模块时打开 |

## 常见问题

**Q1：快速旋转后地图整体歪掉 / 机器人"瞬移"？**
这是 hector 的头号问题：旋转时帧间重叠不足，高斯牛顿收敛到错误解，且没有里程计拉回来。对策按优先级：

1. **控制角速度**：手持/遥控时转弯放慢（TB3 仿真雷达仅 5 Hz，角速度建议 ≤ 0.3 rad/s）；
2. 用高帧率雷达（20–40 Hz 时 hector 才能发挥）；
3. 增大 `map_multi_res_levels`，粗层地图对大位移更宽容；
4. 有 IMU 时配合 `hector_imu_attitude_to_tf` 提供姿态先验；
5. 已经丢了就只能重建：`rostopic pub -1 /syscommand std_msgs/String "reset"` 清图重来。

**Q2：启动后 RViz 有地图但机器人不动 / TF 报错？**
九成是 frame 名不匹配：确认 `base_frame` 与 URDF 一致（TB3 是 `base_footprint`），`rosrun tf view_frames` 或 `rosrun rqt_tf_tree rqt_tf_tree` 检查 TF 树是否从 `map` 连通到雷达 frame。

**Q3：日志刷 `lookupTransform ... extrapolation into the past/future`？**
雷达驱动与主机时间不同步，或 TF 发布延迟。实机上先做 NTP/chrony 对时；rosbag 回放加 `--clock` 并设置 `use_sim_time=true`。

**Q4：长走廊里地图越拉越长？**
走廊方向缺乏几何约束，纯扫描匹配无法感知前进量（"白墙问题"）。hector 无解，需换带里程计约束的方案（gmapping/slam_toolbox），或在走廊布置纸箱等特征物。

**Q5：hector 能配合导航使用吗？**
可以：`pub_map_odom_transform:=true` 且底盘有 odom 时，TF 树与其他 SLAM 一致，move_base 可直接叠加。但生产环境建议只把 hector 用于建图或激光里程计，定位交给 AMCL / slam_toolbox。
