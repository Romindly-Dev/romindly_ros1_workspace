# 25 · 3D LiDAR SLAM

> 中文版 / Chinese: [README.md](../../25_3D激光SLAM/README.md)

This chapter targets multi-beam LiDAR scenarios (16-beam and above mechanical units, Livox solid-state) and covers the principles and hands-on practice of three representative 3D SLAM solutions: A-LOAM (pure LiDAR feature-based method), LIO-SAM (factor-graph tightly-coupled), and FAST-LIO (iterated Kalman tightly-coupled). After finishing this chapter, you should be able to run all three solutions on Romindly Mind using public datasets, understand their respective sensor requirements and parameter systems, and select the right one for your hardware.

## Prerequisites

- Completed [20_2D Mapping](../20_2d_slam/README.md), [30_Localization](../30_localization/README.md), and [40_Navigation](../40_navigation/README.md) (understanding of the SLAM problem definition, the TF tree, and rosbag playback)
- You can study this chapter without a physical 3D LiDAR: all three solutions provide official public rosbags — see [01 Overview and Setup](./01_overview_and_setup.md) for how to obtain them

## Chapter Contents

| No. | Document | Content |
| --- | --- | --- |
| 01 | [3D LiDAR SLAM Overview and Setup](./01_overview_and_setup.md) | 2D vs 3D, technical lineage, point cloud basics, consolidated dependency installation (ceres/GTSAM/Livox-SDK), repository cloning and public datasets |
| 02 | [A-LOAM Hands-On](./02_aloam.md) | LOAM edge/planar features, two-level optimization, running the KITTI/NSH datasets, observing drift |
| 03 | [LIO-SAM Hands-On](./03_lio_sam.md) | Four factor types in the factor graph, 9-axis IMU and ring/time field requirements, params.yaml explained, saving maps with savePCD |
| 04 | [FAST-LIO Hands-On](./04_fast_lio.md) | Iterated error-state Kalman filter + ikd-Tree, velodyne/livox dual data sources, edge-unit deployment tuning |

## Three Solutions at a Glance

| Solution | Sensor requirements | Requires 9-axis IMU | Loop closure | CPU usage | Map output format | Recommended scenarios |
| --- | --- | --- | --- | --- | --- | --- |
| A-LOAM | LiDAR only (multi-beam mechanical, e.g. VLP-16/HDL-32/64) | No IMU needed | None | Medium (two-level Ceres optimization; odometry rate drops under aggressive motion) | Real-time point cloud map in rviz (no built-in save service) | Teaching/introductory use, understanding the LOAM feature-based method; smooth, low-speed platforms |
| LIO-SAM | LiDAR + IMU (point cloud must carry ring/time fields; IMU ≥200 Hz) | **Required** (relies on roll/pitch/yaw for attitude initialization) | Yes (ICP loop closure + global factor-graph optimization, optional GPS factor) | Medium-high (iSAM2 backend + loop closure thread, tunable via `numberOfCores`) | Keyframe point cloud stitching, export PCD via the `savePCD`/`save_map` service | Large-scale mapping, survey-grade tasks needing loop closure and GPS fusion |
| FAST-LIO | LiDAR + IMU (6-axis suffices; native support for Livox CustomMsg and standard PointCloud2) | Not required (6-axis is enough) | None | Low (ikd-Tree incremental map; feature extraction disabled by default) | Saves the full PCD on exit (`pcd_save_en`) | Compute-constrained edge/airborne platforms, scenarios where real-time odometry comes first |

> Romindly Mind configuration advice: the N150 can run dataset playback for all three solutions, but there is little headroom left once LIO-SAM's loop closure is enabled; **for 3D SLAM the N305 variant (8 cores) is recommended — real-time mapping and loop-closure optimization run with much more margin**.

## Common Conventions

- The workspace is uniformly `~/ws_romindly`; the four related repositories (A-LOAM, LIO-SAM, FAST_LIO, livox_ros_driver) are already under `src/` via the clone script from [00 Environment Setup](../00_setup/README.md).
- Demo datasets are all downloaded to the `~/bags/` directory.
- Each of the three hands-on articles first explains the principles, then runs the dataset, and finally covers real-hardware integration; **no physical LiDAR needs to be connected** during dataset playback.
