# 03 RealSense 深度相机接入

> English version: [03_realsense.md](../en/45_sensor_drivers/03_realsense.md)

## 目标

- 用 apt 二进制包在 Romindly Mind 上装好 `realsense2_camera`，启动 D435i/D455 等 D400 系列相机。
- 验证彩色/深度/IMU 话题，开启对齐深度与点云输出。
- 把深度点云接入 [40 章 costmap](../40_导航/02_costmap代价地图调参.md) 的障碍层，让导航能躲开 2D 雷达扫不到的低矮/悬空障碍物。

## 硬件连接

- RealSense 必须走 **USB 3.x**：使用相机原装 USB-C 线接 Romindly Mind 的 Type-C 口。三个 Type-C 口中优先选支持 USB3 速率的口（接好后可用 `lsusb -t` 确认协商到 5000M）。
- 不要串接无供电的 HUB 或超过 2m 的延长线；USB2 链路下深度流分辨率/帧率会被严重限制甚至打不开。
- 连接后确认识别：

```bash
lsusb | grep -i intel        # 应出现 Intel Corp. RealSense 设备
lsusb -t | grep -i 5000      # 确认挂在 5000M（USB3）链路上
```

## 驱动安装（apt 二进制，推荐）

```bash
sudo apt update
sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

说明：

- `librealsense2` 运行库会作为 apt 依赖**自动带入**，无需单独安装；`realsense2-description` 提供相机 URDF（供 TF/仿真用）。
- ROS 仓库自带的 librealsense2 不含内核 patch 与 `realsense-viewer` 等工具。若需要官方工具链或升级固件，可添加 **Intel 官方源** 安装 `librealsense2-utils`、`librealsense2-dkms`（方法见 [librealsense 官方文档](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)），与 apt 版驱动可共存但注意版本一致。
- 源码 fork [Romindly-Dev/realsense-ros](https://github.com/Romindly-Dev/realsense-ros)（`ros1-legacy` 分支）仅供阅读参数实现，教学流程无需编译。

## 启动与验证

```bash
roslaunch realsense2_camera rs_camera.launch
```

常用变体：

```bash
# 深度对齐到彩色（做 RGB-D 应用/标注时常用）
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# 开启点云输出（filters:=pointcloud）
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

# D435i/D455 开启 IMU 并合并成一路话题
roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation
```

验证：

```bash
rostopic hz /camera/color/image_raw          # 默认约 30 Hz
rostopic hz /camera/depth/image_rect_raw
rqt_image_view                                # 下拉选彩色/深度话题肉眼确认
rviz                                          # Fixed Frame 设 camera_link，PointCloud2 订阅 /camera/depth/color/points
```

## 话题清单（默认 rs_camera.launch，前缀 /camera）

| 话题 | 消息类型 | 说明 | 开启条件 |
|------|----------|------|----------|
| `/camera/color/image_raw` | Image | 彩色图 | 默认 |
| `/camera/color/camera_info` | CameraInfo | 彩色内参 | 默认 |
| `/camera/depth/image_rect_raw` | Image (16UC1, mm) | 深度图 | 默认 |
| `/camera/aligned_depth_to_color/image_raw` | Image | 对齐到彩色的深度图 | `align_depth:=true` |
| `/camera/depth/color/points` | PointCloud2 | 彩色点云 | `filters:=pointcloud` |
| `/camera/infra1(2)/image_rect_raw` | Image | 左/右红外图 | `enable_infra1/2:=true` |
| `/camera/gyro/sample`、`/camera/accel/sample` | Imu | 陀螺仪/加速度计（D435i/D455） | `enable_gyro/accel:=true` |
| `/camera/imu` | Imu | 合并后的 IMU | 另加 `unite_imu_method:=...` |

其他常用参数：`depth_width/depth_height/depth_fps`、`color_width/color_height/color_fps`（降低分辨率是缓解 USB 带宽/CPU 压力最有效的手段）、`serial_no`（多相机时按序列号区分，`rs-enumerate-devices` 或启动日志可查）。

## 对接 40 章导航：深度点云进 costmap 障碍层

2D 雷达只能看到安装高度那一个平面，桌沿、悬空横杆、地面小物体全是盲区。把 RealSense 点云喂给 costmap 的 voxel layer 可以补上这块。在 [40 章 costmap 调参](../40_导航/02_costmap代价地图调参.md) 的 `local_costmap` 配置里给 obstacle layer 增加一个点云源：

```yaml
# local_costmap_params.yaml（片段）
plugins:
  - {name: obstacle_layer, type: "costmap_2d::VoxelLayer"}
  - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

obstacle_layer:
  observation_sources: laser_scan camera_cloud
  laser_scan:
    topic: /scan
    data_type: LaserScan
    marking: true
    clearing: true
  camera_cloud:
    topic: /camera/depth/color/points
    data_type: PointCloud2
    marking: true
    clearing: true
    min_obstacle_height: 0.05   # 滤掉地面点
    max_obstacle_height: 1.5    # 高于机器人本体的不管
    obstacle_range: 3.0         # RealSense 有效测距内
    raytrace_range: 3.5
  z_resolution: 0.1
  z_voxels: 16
  publish_voxel_map: true
```

要点：

- 需发布 `base_link → camera_link` 静态 TF（安装位姿实测填入），否则点云无法投影进 costmap。
- 点云频率高、点数大，边缘单元上建议把深度流降到 `640x480@15` 并保留 `filters:=pointcloud`；必要时先过 `voxel_grid` 降采样再进 costmap。
- `min_obstacle_height` 过小会把地面当障碍（相机俯仰角误差、地面反光都会引入地面点），从 0.05 起调。

## 常见问题

**1. `No RealSense devices were found`**
- 线缆/接口不是 USB3 或接触不良（换原装线、换口后 `lsusb` 复查）；
- librealsense 版本与相机固件差距过大——用 Intel 源装 `librealsense2-utils` 后跑 `rs-fw-update -l` 查看/升级固件；
- 刚插入即启动可能枚举未完成，等 2~3 秒重试。

**2. 启动后频繁 `Frames didn't arrive within 5 seconds` / 图像断流**
- USB 带宽不足：降低分辨率帧率（`depth_fps:=15 color_fps:=15`），关掉不用的红外流（`enable_infra1:=false enable_infra2:=false`）；
- 与其他高带宽 USB 设备（如另一台相机）共用同一控制器，换到不同 Type-C 口。

**3. `align_depth` 后 CPU 占用明显升高**
- 对齐在主机侧计算，属正常现象；只做避障不需要对齐，用原始深度点云即可。

**4. 点云话题存在但 RViz 不显示**
- 未加 `filters:=pointcloud`；或 Fixed Frame 与相机 TF 不连通（先加静态 TF）。

**5. IMU 话题没有数据**
- 只有 D435i/D455 等带 IMU 型号支持；需显式 `enable_gyro:=true enable_accel:=true`。

---

上一篇：[02 Velodyne 与 Livox 接入](02_Velodyne与Livox接入.md) | 下一篇：[04 udev 规则与设备管理](04_udev规则与设备管理.md)
