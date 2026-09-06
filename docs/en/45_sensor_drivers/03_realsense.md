# 03 RealSense

> 中文版 / Chinese: [03_RealSense深度相机接入.md](../../45_传感器驱动/03_RealSense深度相机接入.md)

## Goals

- Install `realsense2_camera` on Romindly Mind from apt binary packages and start a D400-series camera such as the D435i/D455.
- Verify the color/depth/IMU topics and enable aligned depth and point cloud output.
- Feed the depth point cloud into the obstacle layer of the [Chapter 40 costmap](../40_navigation/02_costmap_tuning.md), so navigation can avoid low-lying/overhanging obstacles that a 2D LiDAR cannot see.

## Hardware Connection

- RealSense requires **USB 3.x**: use the camera's original USB-C cable connected to a Type-C port on Romindly Mind. Among the three Type-C ports, prefer one that supports USB3 speed (after connecting, confirm with `lsusb -t` that it negotiated 5000M).
- Do not daisy-chain unpowered hubs or extension cables longer than 2 m; on a USB2 link, the depth stream's resolution/frame rate is severely limited or fails to open entirely.
- After connecting, confirm the device is recognized:

```bash
lsusb | grep -i intel        # an Intel Corp. RealSense device should appear
lsusb -t | grep -i 5000      # confirm it is on a 5000M (USB3) link
```

## Driver Installation (apt binaries, recommended)

```bash
sudo apt update
sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

Notes:

- The `librealsense2` runtime library is **pulled in automatically** as an apt dependency, no separate install needed; `realsense2-description` provides the camera URDF (for TF/simulation).
- The librealsense2 shipped in the ROS repository does not include the kernel patch or tools such as `realsense-viewer`. If you need the official toolchain or firmware upgrades, add the **official Intel repository** and install `librealsense2-utils` and `librealsense2-dkms` (see the [official librealsense documentation](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)); it can coexist with the apt driver, but keep the versions consistent.
- The source fork [Romindly-Dev/realsense-ros](https://github.com/Romindly-Dev/realsense-ros) (`ros1-legacy` branch) is only for reading the parameter implementation; the teaching workflow does not require building it.

## Launch and Verification

```bash
roslaunch realsense2_camera rs_camera.launch
```

Common variants:

```bash
# align depth to color (common for RGB-D applications/annotation)
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# enable point cloud output (filters:=pointcloud)
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

# D435i/D455: enable the IMU and merge into a single topic
roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation
```

Verification:

```bash
rostopic hz /camera/color/image_raw          # about 30 Hz by default
rostopic hz /camera/depth/image_rect_raw
rqt_image_view                                # select the color/depth topic from the dropdown for a visual check
rviz                                          # set Fixed Frame to camera_link, PointCloud2 subscribed to /camera/depth/color/points
```

## Topic List (default rs_camera.launch, prefix /camera)

| Topic | Message Type | Description | Enabled By |
|------|----------|------|----------|
| `/camera/color/image_raw` | Image | Color image | Default |
| `/camera/color/camera_info` | CameraInfo | Color intrinsics | Default |
| `/camera/depth/image_rect_raw` | Image (16UC1, mm) | Depth image | Default |
| `/camera/aligned_depth_to_color/image_raw` | Image | Depth image aligned to color | `align_depth:=true` |
| `/camera/depth/color/points` | PointCloud2 | Colored point cloud | `filters:=pointcloud` |
| `/camera/infra1(2)/image_rect_raw` | Image | Left/right infrared images | `enable_infra1/2:=true` |
| `/camera/gyro/sample`, `/camera/accel/sample` | Imu | Gyroscope/accelerometer (D435i/D455) | `enable_gyro/accel:=true` |
| `/camera/imu` | Imu | Merged IMU | additionally `unite_imu_method:=...` |

Other common parameters: `depth_width/depth_height/depth_fps`, `color_width/color_height/color_fps` (lowering resolution is the most effective way to relieve USB bandwidth/CPU pressure), `serial_no` (to distinguish multiple cameras by serial number, obtainable via `rs-enumerate-devices` or the startup log).

## Integrating with Chapter 40 Navigation: Depth Point Cloud into the Costmap Obstacle Layer

A 2D LiDAR only sees the single plane at its mounting height — table edges, overhanging bars, and small objects on the floor are all blind spots. Feeding the RealSense point cloud into the costmap's voxel layer fills this gap. In the `local_costmap` configuration of [Chapter 40 costmap tuning](../40_navigation/02_costmap_tuning.md), add a point cloud source to the obstacle layer:

```yaml
# local_costmap_params.yaml (excerpt)
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
    min_obstacle_height: 0.05   # filter out ground points
    max_obstacle_height: 1.5    # ignore anything above the robot body
    obstacle_range: 3.0         # within RealSense's effective range
    raytrace_range: 3.5
  z_resolution: 0.1
  z_voxels: 16
  publish_voxel_map: true
```

Key points:

- You must publish a `base_link → camera_link` static TF (fill in the measured mounting pose); otherwise the point cloud cannot be projected into the costmap.
- The point cloud is high-rate and dense; on an edge unit, it is recommended to lower the depth stream to `640x480@15` while keeping `filters:=pointcloud`; if necessary, downsample through `voxel_grid` before it enters the costmap.
- Setting `min_obstacle_height` too low causes the ground to be treated as an obstacle (camera pitch error and floor reflections both introduce ground points); start tuning from 0.05.

## Common Issues

**1. `No RealSense devices were found`**
- The cable/port is not USB3 or the connection is loose (swap to the original cable or a different port, then re-check with `lsusb`);
- The librealsense version differs too much from the camera firmware — install `librealsense2-utils` from the Intel repository and run `rs-fw-update -l` to inspect/upgrade the firmware;
- Starting immediately after plugging in may find enumeration incomplete; wait 2~3 seconds and retry.

**2. Frequent `Frames didn't arrive within 5 seconds` / image stream drops after startup**
- Insufficient USB bandwidth: lower the resolution and frame rate (`depth_fps:=15 color_fps:=15`), disable unused infrared streams (`enable_infra1:=false enable_infra2:=false`);
- Sharing the same controller with another high-bandwidth USB device (e.g. another camera); move to a different Type-C port.

**3. CPU usage rises noticeably with `align_depth`**
- Alignment is computed on the host side, which is expected; for obstacle avoidance alone, alignment is unnecessary — use the raw depth point cloud.

**4. Point cloud topic exists but RViz shows nothing**
- `filters:=pointcloud` was not added; or the Fixed Frame is not connected to the camera TF (add the static TF first).

**5. No data on the IMU topics**
- Only IMU-equipped models such as the D435i/D455 support this; you must explicitly set `enable_gyro:=true enable_accel:=true`.

---

Previous: [02 Velodyne and Livox](02_velodyne_and_livox.md) | Next: [04 udev Rules and Device Management](04_udev_and_device_management.md)
