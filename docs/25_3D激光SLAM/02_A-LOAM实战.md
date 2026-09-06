# 02 · A-LOAM 实战

## 目标

- 直观理解 LOAM 的两个核心思想：边缘/平面特征、scan-to-scan + scan-to-map 两级优化
- 编译 A-LOAM 并用公开数据集（NSH 或 KITTI）跑通完整建图流程
- 掌握三个节点的分工与话题关系，观察纯激光里程计的累积漂移

## 一、原理直观版

LOAM（Lidar Odometry and Mapping）的聪明之处在于：**不拿几十万个点硬配准，而是只挑"有个性"的点**。

- **边缘点（corner/sharp）**：落在物体棱边上的点，其左右邻居点距离变化剧烈（曲率大）——如墙角、柱子边缘；
- **平面点（surf/flat）**：落在平整表面上的点，邻居们几乎共面（曲率小）——如地面、墙面。

配准时，边缘点去匹配上一帧/地图中的"线"，平面点去匹配"面"，构成点到线、点到面的距离残差，交给 Ceres 做非线性最小二乘。

第二个核心思想是**两级优化，分频运行**：

```mermaid
graph LR
    A["原始点云<br/>/velodyne_points 10Hz"] --> B["scanRegistration<br/>按曲率提取边缘/平面特征"]
    B --> C["laserOdometry 10Hz<br/>scan-to-scan 粗配准<br/>相邻两帧特征匹配"]
    C --> D["laserMapping 约5Hz<br/>scan-to-map 精配准<br/>当前帧对局部特征地图"]
    D --> E["精确位姿 + 全局点云地图"]
    C -. 高频粗位姿 .-> E
```

- **scan-to-scan（laserOdometry）**：相邻两帧特征互相配准，快但误差会逐帧累积；
- **scan-to-map（laserMapping）**：当前帧对已建好的局部特征地图配准，慢一些但准得多，同时不断修正里程计的漂移。

高频粗位姿保证实时性，低频精位姿保证精度——这个"快慢双层"架构被后来几乎所有激光 SLAM 继承。A-LOAM 是 LOAM 的 Ceres 重写版，代码干净、没有复杂手推雅可比，非常适合作为 3D SLAM 的第一课。

## 二、环境准备

依赖只有 Ceres 与 PCL（PCL 随 ROS 已装）：

```bash
sudo apt install libceres-dev
cd ~/ws_romindly && catkin_make --pkg aloam_velodyne
source devel/setup.bash
```

数据集：从 `A-LOAM/README.md` 第 3 节的 Google Drive 链接下载 `nsh_indoor_outdoor.bag`（VLP-16 采集的室内外混合场景，约 3 GB），放到 `~/bags/`。

## 三、分步操作：跑通 NSH 数据集

**终端 1** — 启动 A-LOAM 三节点 + rviz：

```bash
source ~/ws_romindly/devel/setup.bash
roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
```

预期输出：三个节点启动，rviz 打开并加载 `aloam_velodyne.rviz` 配置，终端等待点云输入。

**终端 2** — 回放数据：

```bash
rosbag play ~/bags/nsh_indoor_outdoor.bag
```

几秒后 rviz 中应出现：白色当前帧点云、彩色累积地图、绿色（odometry）与红色（mapping）两条轨迹逐渐延伸。终端 1 持续打印每帧的处理耗时（`whole laserOdometry time` 等）。

**验证话题连通**（终端 3）：

```bash
rostopic hz /velodyne_points /laser_odom_to_init /aft_mapped_to_init
# 预期：/velodyne_points ≈10Hz，/laser_odom_to_init ≈10Hz，/aft_mapped_to_init ≈5Hz
```

### KITTI 数据集（可选）

下载 KITTI Odometry 数据集后，编辑 `A-LOAM/launch/kitti_helper.launch` 中的 `dataset_folder`、`sequence_number`（并可设 `to_bag` 转存 bag），然后：

```bash
roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch   # KITTI 是 64 线
roslaunch aloam_velodyne kitti_helper.launch            # 另一终端，读取并发布 KITTI 数据
```

## 四、节点、话题与参数详解

### 4.1 三个节点与话题流

| 节点（可执行文件） | 订阅 | 发布（主要） | 作用 |
| --- | --- | --- | --- |
| `ascanRegistration` | `/velodyne_points` | `/laser_cloud_sharp`、`/laser_cloud_less_sharp`、`/laser_cloud_flat`、`/laser_cloud_less_flat`、`/velodyne_cloud_2` | 按线拆分点云、计算曲率、提取两级边缘/平面特征 |
| `alaserOdometry` | 上述特征话题 | `/laser_odom_to_init`（10 Hz 位姿）、`/laser_cloud_corner_last`、`/laser_cloud_surf_last`、`/laser_odom_path` | scan-to-scan 帧间里程计 |
| `alaserMapping` | odometry 输出的特征与位姿 | `/aft_mapped_to_init`（精位姿）、`/aft_mapped_to_init_high_frec`（高频插值位姿）、`/laser_cloud_map`、`/velodyne_cloud_registered`、`/aft_mapped_path` | scan-to-map 精配准与地图维护 |

sharp/less_sharp（flat/less_flat）的区别：sharp 是曲率最大的少量点，用作优化的"源"；less_sharp 数量更多，用作下一帧匹配的"目标"，提高找到对应的概率。

### 4.2 参数表（launch 文件内，A-LOAM 参数就这么几个）

| 参数 | VLP_16 默认值 | 说明 |
| --- | --- | --- |
| `scan_line` | 16 | 雷达线数，**必须与数据匹配**（16/32/64），否则按线拆分错乱、特征全错 |
| `minimum_range` | 0.3 | 近距离盲区剔除（米），过滤打在载体自身上的点 |
| `mapping_skip_frame` | 1 | 里程计每 N 帧送 1 帧给 mapping；1=10 Hz 建图，2=5 Hz，算力紧张可调大 |
| `mapping_line_resolution` | 0.2 | 地图边缘特征的体素降采样尺寸（米），越大越省算力、越粗糙 |
| `mapping_plane_resolution` | 0.4 | 地图平面特征的体素降采样尺寸（米），同上 |

参数少正是 A-LOAM 的特点：没有 IMU、没有回环、没有关键帧策略，一切从简——这让它成为读懂 LOAM 论文代码对照的最佳教材，但也意味着它对场景与运动方式更敏感。

### 4.3 rviz 配置要点

自带的 `rviz_cfg/aloam_velodyne.rviz` 已配好，若要手动搭建（或嵌入自己的 rviz 工程），关键项：

- Fixed Frame 设为 `camera_init`（A-LOAM 沿用 LOAM 的相机系命名，`camera_init` 即建图原点）；
- 地图：`PointCloud2` 显示 `/laser_cloud_map`（特征地图，稀疏）或 `/velodyne_cloud_registered`（配准后完整点云，稠密），Decay Time 调大可累积显示；
- 轨迹：`Path` 显示 `/laser_odom_path`（绿色，粗）与 `/aft_mapped_path`（红色，精），对比两条线可直观看出 mapping 对 odometry 的修正量。

## 五、实机接入要点

接入实体 Velodyne（参考 [45 传感器驱动](../45_传感器驱动/README.md)）时只需三步：

```bash
sudo apt install ros-noetic-velodyne            # 官方驱动（本套 fork 的 velodyne 仓库亦可源码编译）
roslaunch velodyne_pointcloud VLP16_points.launch   # 发布 /velodyne_points
roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
```

注意事项：

- `scan_line` 与实机线数一致；32 线用 `aloam_velodyne_HDL_32.launch`；
- 载体上有遮挡结构（支架、外壳）时把 `minimum_range` 调大到遮挡半径之外；
- A-LOAM 无 IMU 输入，**避免急转弯与颠簸路面**，运动越平缓效果越好——这条限制正是引出后两篇 LIO 方案的现实动机。

## 六、观察无回环的漂移

A-LOAM 没有回环检测：走一大圈回到起点时，累积误差不会被修正。用 NSH 数据集能直接观察到——bag 末段回到出发的室内区域，rviz 中新旧点云会出现明显"重影"（墙面错层）。这不是 bug，而是纯里程计式 SLAM 的本质缺陷，也是下一篇 LIO-SAM 引入因子图 + 回环的动机。

另外注意 A-LOAM 不提供地图保存服务，rviz 里看到的地图关掉就没了；需要保存可用 `rosbag record /velodyne_cloud_registered` 记录配准后点云，或用 `pcl_ros` 的 `pointcloud_to_pcd` 工具落盘。

## 常见问题

**Q1：rviz 里完全没有点云？**
检查 bag 的点云话题名：`rosbag info xxx.bag`。A-LOAM 固定订阅 `/velodyne_points`，话题不同时用重映射回放：`rosbag play xxx.bag /your_topic:=/velodyne_points`。

**Q2：地图很快飘掉/断层？**
最常见原因是 `scan_line` 与数据线数不符（如用 VLP_16 的 launch 放 64 线 KITTI 数据）。其次是运动过快或场景几何特征太少（长直走廊、空旷广场），纯激光方案在这类退化场景无解，需要 IMU（见后两篇）。

**Q3：终端打印的处理时间越来越长？**
地图特征点随运行增多，mapping 耗时上升。可调大 `mapping_line_resolution`/`mapping_plane_resolution` 或把 `mapping_skip_frame` 设为 2。

**Q4：`error while loading shared libraries: libceres.so`？**
Ceres 是编译后新装/升级的，重新 `catkin_make` 一次并确认 `sudo ldconfig`。

下一篇：[03 LIO-SAM 实战](./03_LIO-SAM实战.md)
