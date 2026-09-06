# 05 · cartographer 实战

## 目标

- 用 apt 安装 cartographer_ros 并理解其 lua 配置体系
- 在 TB3 仿真中完成在线建图，掌握 pbstream 的保存与转换
- 会用 offline node 对 rosbag 做离线建图
- 建立"先关回环调前端，再开回环调后端"的调参方法论

## 安装方式

**统一使用 apt 安装**：

```bash
sudo apt install ros-noetic-cartographer-ros ros-noetic-cartographer-rviz
```

不要把 cartographer 源码放进主工作空间：它不用 catkin_make 而要 `catkin_make_isolated`，且依赖需要单独编译的 abseil-cpp，会拖累整个工作空间的构建。需要阅读源码时 clone 我们的 fork 到独立目录即可（只读不编译）：

```bash
git clone https://github.com/Romindly-Dev/cartographer_ros ~/reading/cartographer_ros
```

## 原理简介

cartographer（Google 出品）是图优化路线中工程化最彻底的实现，分两层：

- **前端（Local SLAM，`TRAJECTORY_BUILDER_2D`）**：连续的扫描经体素滤波、Ceres 扫描匹配后插入当前**子图（submap）**；每张子图攒够 `num_range_data` 帧就封存，开始下一张。误差在子图内很小，但子图之间会累积漂移。
- **后端（Global SLAM，`POSE_GRAPH`）**：所有扫描与所有封存子图之间用**分支定界**算法高效搜索回环约束，周期性做全局优化（SPA），把子图重新摆正。

这种"子图为单位的回环"使它在超大场景下依然稳健，也是它 CPU 占用偏高的原因（后台持续跑约束搜索）。

## lua 配置体系

cartographer 不用 rosparam，配置全部写在 lua 文件里，通过 `-configuration_directory` + `-configuration_basename` 传给节点。一份典型配置的结构：

```lua
include "map_builder.lua"          -- 引入默认值（安装于 /opt/ros/noetic/share/cartographer/configuration_files/）
include "trajectory_builder.lua"

options = {                        -- ROS 接口层选项（cartographer_ros 消费）
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",     -- 位姿估计参考系：有 IMU 必须设为 IMU 系
  published_frame = "odom",        -- cartographer 发布 map→published_frame
  odom_frame = "odom",
  provide_odom_frame = false,      -- 底盘已有 odom TF 时设 false
  use_odometry = true,             -- 订阅 /odom 作为前端先验
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true      -- 算法层：覆盖默认值

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
```

层级关系：`options` 表是 ROS 桥接层（订阅什么、发布什么 TF）；`MAP_BUILDER` / `TRAJECTORY_BUILDER_2D` / `POSE_GRAPH` 是算法层，`include` 引入默认值后按需逐项覆盖。

## 分步操作

每个终端先 `export TURTLEBOT3_MODEL=burger`。

### 1. 启动仿真与 cartographer

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

TB3 快捷方式（若你安装的 `turtlebot3_slam` 版本仍保留 cartographer 配置）：

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer
```

若提示找不到 launch/lua（新版 turtlebot3_slam 移除了 cartographer 支持），用通用方式：把上节 lua 保存为 `~/carto_ws/config/tb3_2d.lua`，然后自建 launch：

```xml
<launch>
  <node pkg="cartographer_ros" type="cartographer_node" name="cartographer_node"
        args="-configuration_directory $(env HOME)/carto_ws/config
              -configuration_basename tb3_2d.lua" output="screen">
    <remap from="scan" to="/scan"/>
    <remap from="odom" to="/odom"/>
    <remap from="imu"  to="/imu"/>
  </node>
  <node pkg="cartographer_ros" type="cartographer_occupancy_grid_node"
        name="cartographer_occupancy_grid_node" args="-resolution 0.05"/>
</launch>
```

注意：`/map` 话题由独立的 `cartographer_occupancy_grid_node` 光栅化发布，忘启动它则 RViz 无地图（但 TF 与 submap 正常）。

### 2. 遥控、观察与保存

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun rviz rviz   # Fixed Frame=map；也可用 cartographer_rviz 的 Submaps 显示项观察子图
```

绕场一圈，注意回环触发时子图整体微调。保存分两条线：

```bash
# A. 通用静态图（供导航）：
rosrun map_server map_saver -f ~/maps/tb3_world_carto

# B. cartographer 原生状态（pbstream，含完整位姿图，供纯定位/续跑）：
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '/home/iot/maps/tb3_world_carto.pbstream', include_unfinished_submaps: true}"

# pbstream 也能转出 PGM+yaml：
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename ~/maps/tb3_world_carto.pbstream -map_filestem ~/maps/tb3_world_carto
```

### 3. 离线建图（rosbag + offline node）

现场只录包、回办公室精细建图，是 cartographer 的推荐工作流：

```bash
# 现场录包（激光+里程计+IMU+TF）：
rosbag record -O tb3_run1.bag /scan /odom /imu /tf /tf_static

# 离线建图（以最快速度跑完整个包，自动生成 <bag名>.pbstream）：
rosrun cartographer_ros cartographer_offline_node \
    -configuration_directory ~/carto_ws/config \
    -configuration_basename tb3_2d.lua \
    -bag_filenames /home/iot/tb3_run1.bag
```

离线节点不依赖实时性，可反复用不同参数重跑同一个包做对比——这正是下节调参思路的基础。录包时**不要**同时跑别的 SLAM，避免 bag 里混入 `map→odom` TF（或回放时用 `tf` 过滤）。

## 参数详解（关键项）

默认值为 cartographer 安装目录 `configuration_files/*.lua` 中的出厂值：

| 参数 | 默认值 | 作用 | 调参建议 |
| --- | --- | --- | --- |
| `options.use_odometry` | false | 订阅 /odom 给前端做运动先验 | 有轮式里程计务必 true，前端稳定性大幅提升 |
| `options.tracking_frame` | — | 位姿估计参考坐标系 | 用 IMU 时必须设 IMU frame（TB3 为 `imu_link`），否则设 base 系 |
| `options.provide_odom_frame` | — | 由 cartographer 自己发布 odom 系 | 底盘已有 odom TF 设 false，否则 TF 冲突 |
| `TRAJECTORY_BUILDER_2D.use_imu_data` | true | 前端使用 IMU 姿态 | 无 IMU 必须显式设 false，否则一直等 IMU 不出图 |
| `TRAJECTORY_BUILDER_2D.num_accumulated_range_data` | 1 | 攒几次激光消息拼成一帧再匹配 | 单线雷达保持 1；多回波/分段发布的雷达按驱动设置 |
| `TRAJECTORY_BUILDER_2D.min_range / max_range` | 0. / 30. | 有效激光距离 | 按雷达设置，TB3 为 0.12 / 3.5 |
| `TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching` | false | 匹配前先做相关性暴力搜索 | 里程计差/无 IMU 时打开，抗打滑，CPU 增加 |
| `TRAJECTORY_BUILDER_2D.submaps.num_range_data` | 90 | 每张子图包含的帧数 | 子图应小到不受漂移影响、大到能独立辨识；慢速小车可降至 35–60 |
| `TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians` | ~0.017 (1°) | 运动滤波：小于该运动的帧被丢弃 | 静止时地图发糊可调小 |
| `POSE_GRAPH.optimize_every_n_nodes` | 90 | 每插入 n 个节点做一次全局优化；**设 0 = 关闭回环/后端** | 调参第一步设 0；恢复时建议 ≈ num_range_data 的 1–2 倍 |
| `POSE_GRAPH.constraint_builder.min_score` | 0.55 | 回环约束的最低匹配分 | 误回环（地图拉花）提高到 0.65+ |
| `POSE_GRAPH.constraint_builder.sampling_ratio` | 0.3 | 参与回环搜索的节点采样率 | 降低省 CPU、回环变少 |

## 调参思路：先关回环调前端，再开回环

回环优化会掩盖前端问题，混在一起调无从下手。标准流程：

1. **关后端**：`POSE_GRAPH.optimize_every_n_nodes = 0`，用 offline node 跑同一个 bag；
2. **调前端**：目标是"不靠回环也基本不重影"——依次核对 TF/时间戳、`use_odometry`、距离范围，再调 `use_online_correlative_scan_matching`、ceres 权重、`submaps.num_range_data`；
3. **开后端**：恢复 `optimize_every_n_nodes = 90`，只调回环相关（`min_score`、`sampling_ratio`、优化频率），观察回环是否正确触发、有无误回环；
4. 每次只改一个参数，用同一 bag 对比结果。

## 常见问题

**Q1：节点启动后无任何地图输出，也不报错？**
最常见是 `use_imu_data = true` 但没有 IMU 数据——cartographer 会静默等待。无 IMU 时设 false；有 IMU 检查话题重映射与 `tracking_frame`。

**Q2：报 `Check failed: ... frame` 或 TF 超时？**
`tracking_frame` / `published_frame` 与实际 TF 树不符，或 URDF 未加载。cartographer 对 TF 与时间戳的校验远比 gmapping 严格，实机务必对时。

**Q3：地图整体很好但局部有细小锯齿？**
前端每帧匹配的正常噪声。可降低分辨率预期，或调小 `motion_filter` 阈值、提高 ceres 匹配权重；导航层面通常无需处理。

**Q4：CPU 占用过高？**
按序尝试：降低 `POSE_GRAPH.constraint_builder.sampling_ratio`（0.3→0.1）、加大 `optimize_every_n_nodes`、关闭 `use_online_correlative_scan_matching`、加大 motion_filter 阈值。

**Q5：想在 cartographer 里做纯定位？**
lua 中加 `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }`，启动时用 `-load_state_filename xxx.pbstream`。不过 ROS1 生产定位我们更推荐 AMCL 或 slam_toolbox localization，见 [30_定位](../../30_定位/README.md)。
