# 45 Sensor Drivers

> 中文版 / Chinese: [README.md](../../45_传感器驱动/README.md)

The previous chapters (20 2D Mapping, 25 3D LiDAR SLAM, 30 Localization, 40 Navigation) can mostly be completed in simulation or on public datasets. This chapter tackles the first step of moving "from simulation to a real robot": connecting real sensors to the edge computing unit **Romindly Mind** (Ubuntu 20.04 / ROS Noetic / x86_64), installing the drivers, verifying the topics, and pinning stable device names — laying the groundwork for running mapping, localization, and navigation on real hardware.

## Chapter Contents

| Section | Sensor / Topic | Key Deliverables |
|------|--------------|----------|
| [01 RPLIDAR](01_rplidar.md) | Slamtec 2D LiDAR (A1/A2/A3/S/C series) | `/scan` topic, feeding gmapping / slam_toolbox in Chapter 20 |
| [02 Velodyne and Livox](02_velodyne_and_livox.md) | 3D mechanical (VLP-16) and 3D solid-state (Livox) | `/velodyne_points`, `/livox/lidar`, feeding 3D SLAM in Chapter 25 |
| [03 RealSense](03_realsense.md) | Intel RealSense D400 series | Color/depth/point cloud/IMU topics, feeding the costmap obstacle layer in Chapter 40 |
| [04 udev Rules and Device Management](04_udev_and_device_management.md) | Pinned serial device names, static IP for Ethernet ports, prerequisites for boot-time autostart | Stable device names such as `/dev/rplidar`, complementing [robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) |

## Sensor Selection Quick Reference

| Type | Representative Models | Interface | Data Format | Related Chapters | Section |
|------|----------|----------|----------|----------|----------|
| 2D LiDAR | Slamtec RPLIDAR A1/A2/A3/S/C | USB serial (CP210x to `/dev/ttyUSB*`) | `sensor_msgs/LaserScan` | [20 2D Mapping](../20_2d_slam/README.md), [30 Localization](../30_localization/README.md), [40 Navigation](../40_navigation/README.md) | 01 |
| 3D mechanical LiDAR | Velodyne VLP-16 | Ethernet (UDP, LiDAR defaults to 192.168.1.201) | `sensor_msgs/PointCloud2` | [25 3D LiDAR SLAM](../25_3d_lidar_slam/README.md) (A-LOAM / LIO-SAM / FAST-LIO2) | 02 |
| 3D solid-state LiDAR | Livox Horizon / Avia / Mid-40 | Ethernet (identified by broadcast code) | `PointCloud2` or Livox `CustomMsg` | [25 3D LiDAR SLAM](../25_3d_lidar_slam/README.md) (FAST-LIO2 preferred) | 02 |
| Depth camera | Intel RealSense D435i / D455 | USB 3.x (Type-C connector) | Color/depth images, point cloud, IMU | [40 Navigation](../40_navigation/README.md) (costmap obstacle layer), vision-based extensions | 03 |

Rules of thumb for sensor selection:

- **Indoor robot, limited budget, getting the 2D pipeline running first** → RPLIDAR (Section 01); paired with gmapping / slam_toolbox it is sufficient.
- **Outdoor or large-scale scenes requiring 3D point cloud maps** → VLP-16 or Livox (Section 02); if you have an IMU and want low compute usage, choose Livox + FAST-LIO2.
- **Navigation needs to avoid low-lying/overhanging obstacles (invisible to a 2D LiDAR)** → add a RealSense (Section 03) and feed the depth point cloud into the costmap.
- When multiple serial devices are attached at once (LiDAR + IMU + base), **Section 04 is mandatory reading** — otherwise `/dev/ttyUSB0/1` drifting after reboot will make your launches fail at random.

## Interface Resource Planning (Romindly Mind)

Romindly Mind provides dual 2.5G Ethernet ports, 3x Type-C, 4x UART, CAN, and GPIO. Typical allocation for this chapter:

- Ethernet port 1: Velodyne / Livox LiDAR (static IP, see Sections 02/04); Ethernet port 2: internet / debugging.
- Type-C: RealSense (requires a USB3 cable and port, see Section 03); RPLIDAR via a USB adapter also goes through Type-C.
- UART / CAN: base and IMU (see Section 04 for pinned device names).

## Prerequisites

- Complete [00 Environment Setup](../00_setup/01_install_ubuntu2004_ros_noetic.md); `catkin_ws` builds successfully.
- You have the corresponding physical sensors; 3D LiDARs additionally require your own power adapter and Ethernet cable.
