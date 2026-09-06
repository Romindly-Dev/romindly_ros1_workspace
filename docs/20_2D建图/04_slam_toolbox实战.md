# 04 · slam_toolbox 实战

> English version: [04_slam_toolbox.md](../en/20_2d_slam/04_slam_toolbox.md)

## 目标

- 理解 slam_toolbox 被定位为"产品化首选"的原因，及其五种工作模式
- 区分 sync / async 两种节点并完成一次仿真建图
- 掌握 `.posegraph` 序列化地图与 `map_saver` 静态地图的区别与各自用途
- 了解纯定位模式，为 [30_定位](../30_定位/README.md) 做铺垫

## 原理简介

slam_toolbox（Steve Macenski 开发，ROS2/Nav2 的默认 SLAM）基于 Karto 的 scan-to-map 前端重写：位姿图 + Ceres 非线性优化后端，带回环检测。相比前两篇的方案，它的工程能力是质变：

- **在线建图**（online sync/async）：常规实时建图；
- **离线建图**（offline）：回灌 rosbag，不丢帧地处理每一帧；
- **持续建图（续建）**：加载 `.posegraph` 后在旧图基础上继续建图——客户现场扩建了一个库区，不必推倒重扫；
- **纯定位**（localization 模式）：在已有位姿图上做"弹性定位"，新扫描只建临时约束不改地图，可替代 AMCL；
- **lifelong**（实验性）：长期运行中增删节点，适应环境变化。

一套包覆盖"建图 → 续建 → 定位"全生命周期，加上回环闭合带来的大场景精度，这就是产品化首选的理由。

**sync vs async 节点**：

- `sync_slam_toolbox_node`（online_sync）：把符合条件的扫描全部排队处理，**不丢帧**，地图质量优先，机器人快速移动时处理可能滞后——适合离线处理和对质量要求高的建图；
- `async_slam_toolbox_node`（online_async）：永远处理**最新一帧**，处理不过来就丢弃旧帧，保证实时性——适合算力有限的机载电脑上边跑边建图。

两者参数完全同构，只是调度策略不同：教学与仿真用 sync，实机部署一般选 async。

源码阅读：`Romindly-Dev/slam_toolbox`（上游 SteveMacenski，noetic 分支）。配置文件在 `slam_toolbox/config/mapper_params_*.yaml`，launch 在 `slam_toolbox/launch/`。

## 分步操作

```bash
sudo apt install ros-noetic-slam-toolbox     # 首次
```

每个终端先 `export TURTLEBOT3_MODEL=burger`。

### 1. 启动仿真与 slam_toolbox

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

新终端（默认配置 `base_frame: base_footprint`、`scan_topic: /scan`，与 TB3 完全匹配，无需修改）：

```bash
roslaunch slam_toolbox online_sync.launch
```

预期输出（节选）：

```text
[ INFO] [...]: Solver plugin loaded: solver_plugins::CeresSolver
Registering sensor: [Custom Described Lidar]
```

实机 / 自定义配置的通用写法（推荐复制官方 yaml 到自己的包再加载，而不是命令行传参）：

```xml
<launch>
  <node pkg="slam_toolbox" type="sync_slam_toolbox_node" name="slam_toolbox" output="screen">
    <rosparam command="load" file="$(find my_robot_bringup)/config/slam_toolbox.yaml"/>
    <param name="base_frame" value="base_link"/>   <!-- 覆盖 yaml，按底盘修改 -->
    <param name="max_laser_range" value="12.0"/>
  </node>
</launch>
```

启动后自检：

```bash
rosnode info /slam_toolbox | grep -A4 Services   # 应看到 save_map / serialize_map 等服务
rosrun tf tf_echo map odom                       # 确认 map->odom 已发布
```

### 2. RViz 与遥控

```bash
rosrun rviz rviz    # Fixed Frame=map，添加 Map(/map) 与 LaserScan(/scan)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

注意默认 `minimum_travel_distance: 0.5` / `minimum_travel_heading: 0.5`——机器人移动 0.5 m 或转 0.5 rad 才纳入新节点，起步阶段地图更新看起来"迟钝"是正常的。绕场一圈回到起点，观察回环闭合瞬间整张地图被"拉正"（日志出现 loop closure 相关信息）。

RViz 可加载专属面板：Panels → Add New Panel → `SlamToolboxPlugin`，可视化完成保存/序列化/暂停等操作。

### 3. 保存地图（两种方式，务必都做）

```bash
# 方式一：静态栅格图（给 map_server/AMCL/move_base 用）
rosrun map_server map_saver -f ~/maps/tb3_world_toolbox
# 等价服务：rosservice call /slam_toolbox/save_map "name: {data: '/home/iot/maps/tb3_world_toolbox'}"

# 方式二：序列化位姿图（给 slam_toolbox 自己续建/纯定位用）
rosservice call /slam_toolbox/serialize_map "filename: '/home/iot/maps/tb3_world_toolbox'"
```

方式二生成 `tb3_world_toolbox.posegraph` + `tb3_world_toolbox.data`。**区别**：`map_saver` 导出的 PGM 是"死"的光栅快照，只能看和定位；`.posegraph` 保留了全部节点、扫描与约束，是"活"的工程文件，可重新打开继续建图或做弹性定位。产品交付建议两种都归档。

### 4. 续建（持续建图）

编辑配置（或用 RViz 面板 Deserialize）：在 `mapper_params_online_sync.yaml` 中取消注释：

```yaml
map_file_name: /home/iot/maps/tb3_world_toolbox   # 不带扩展名
map_start_at_dock: true          # 从建图起点继续；或用 map_start_pose: [x, y, theta]
```

重启 `online_sync.launch`，旧图加载后即可继续扫新区域，也可调用服务 `/slam_toolbox/deserialize_map` 动态加载。

### 5. 纯定位模式（预告）

```bash
roslaunch slam_toolbox localization.launch
```

使用前编辑 `config/mapper_params_localization.yaml`：把 `mode: mapping` 改为 `mode: localization`（注意：源码仓库该文件此行默认仍是 `mapping`，注释里才是 `localization`，**必须手动改**），并设置 `map_file_name` 指向 `.posegraph`。该模式下地图不再被修改，新扫描仅用于弹性匹配定位，支持 `/initialpose`（RViz 2D Pose Estimate）重定位。与 AMCL 的对比与选型放在 30 章展开。

## 参数详解

核对自 `config/mapper_params_online_sync.yaml`（async 同构），只列关键项：

| 参数 | 默认值 | 作用 | 调参建议 |
| --- | --- | --- | --- |
| `mode` | mapping | mapping / localization | 定位部署时改 localization |
| `solver_plugin` | CeresSolver | 后端优化器 | 保持默认 |
| `resolution` | 0.05 | 地图分辨率 (m) | 同其他方案，通用 0.05 |
| `max_laser_range` | 20.0 | 参与建图的激光最大距离 | 按雷达设置，TB3 设 3.5；过大引入噪点 |
| `map_update_interval` | 5.0 (s) | /map 发布周期 | 只影响可视化 |
| `minimum_travel_distance` | 0.5 (m) | 新增图节点的平移阈值 | 小房间可降 0.3；越小图越大、CPU 越高 |
| `minimum_travel_heading` | 0.5 (rad) | 新增图节点的旋转阈值 | 可降 0.25 改善转弯细节 |
| `minimum_time_interval` | 0.5 (s) | 两帧处理的最小时间间隔 | 一般不动 |
| `scan_buffer_size` | 10 | 参与 scan-to-map 匹配的滑动窗口帧数 | 里程计差可加大 |
| `do_loop_closing` | true | 回环开关 | 调前端时可暂时关闭（思路同 cartographer） |
| `loop_search_maximum_distance` | 3.0 (m) | 回环候选搜索半径 | 大场景/漂移大适当加大（配合下项） |
| `loop_match_minimum_chain_size` | 10 | 构成回环所需最小节点链长 | 误回环多则加大，回环触发难则减小 |
| `loop_match_minimum_response_coarse/fine` | 0.35 / 0.45 | 回环匹配响应阈值 | 误回环（地图被拉花）就提高；漏回环就降低 |
| `correlation_search_space_dimension` | 0.5 (m) | 普通匹配搜索窗口 | 里程计差加大到 0.8–1.0 |
| `transform_publish_period` | 0.02 (s) | map→odom TF 周期 | 一般不动 |
| `enable_interactive_mode` | true | RViz 交互式修图（拖动节点） | 生产部署关掉省内存 |
| `stack_size_to_use` | 40000000 | 序列化大地图所需栈空间 | 超大地图序列化崩溃时加大 |
| `debug_logging` | false | 详细日志 | 排障时打开 |

## 常见问题

**Q1：启动即报 TF 错误 / 地图不出？**
确认 `base_frame` 与实际机器人一致（默认 `base_footprint`，很多底盘是 `base_link`），且 `odom→base` TF 存在。slam_toolbox 与 gmapping 一样**必须有里程计**。

**Q2：回环闭合瞬间地图"跳了一下"正常吗？**
正常且正是价值所在——全局优化把累积误差一次性重分配到历史轨迹。若跳完反而更糟（误回环），提高 `loop_match_minimum_response_*` 阈值。

**Q3：serialize 与 map_saver 存的图能互转吗？**
`.posegraph` 随时可以重新光栅化出 PGM（加载后再 map_saver）；反向不行——PGM 丢失了位姿图信息，无法续建。所以**建完图第一时间 serialize**。

**Q4：deserialize 后提示找不到文件？**
`map_file_name` 与服务调用中的路径要用**绝对路径且不带 `.posegraph` 扩展名**；同目录必须同时存在 `.posegraph` 与 `.data` 两个文件。

**Q5：CPU 随建图时间上涨？**
图优化方案的固有特性（节点越多优化越贵）。对策：适当加大 `minimum_travel_distance/heading` 控制节点密度；机载算力弱用 async 节点；超大场景分区建图后用 `merge_maps_kinematic` 合并。
