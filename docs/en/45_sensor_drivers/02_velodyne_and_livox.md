# 02 Velodyne and Livox

> 中文版 / Chinese: [02_Velodyne与Livox接入.md](../../45_传感器驱动/02_Velodyne与Livox接入.md)

## Goals

- Velodyne VLP-16 (3D mechanical): connect directly to Romindly Mind over Ethernet, run `VLP16_points.launch`, and verify `/velodyne_points`.
- Livox (3D solid-state, Horizon/Avia/Mid-40, etc.): install Livox-SDK and `livox_ros_driver`, understand `xfer_format`, and output the CustomMsg required by FAST-LIO.
- Clarify how the output topics of both map onto the inputs of the SLAM approaches in [Chapter 25 3D SLAM](../25_3d_lidar_slam/README.md).

## Velodyne VLP-16

### Hardware Connection and Network Configuration

The VLP-16 is powered through an Interface Box and outputs over an Ethernet cable. Its factory-default IP is **192.168.1.201**, sending UDP data to the broadcast address on port 2368.

Romindly Mind has dual 2.5G Ethernet ports. Typical wiring: **port 1 directly connected to the LiDAR** (static IP), **port 2 connected to the internet/router**, without interfering with each other:

```bash
ip link                      # confirm interface names, e.g. enp1s0 (LiDAR) / enp2s0 (internet)
# temporary configuration (lost on reboot; see Section 04 for netplan persistence)
sudo ip addr add 192.168.1.100/24 dev enp1s0
sudo ip link set enp1s0 up

ping 192.168.1.201           # a successful ping means the physical link is fine
sudo tcpdump -i enp1s0 udp port 2368 -c 5   # capturing packets means the LiDAR is outputting data
```

Note that the static IP should be 192.168.1.x (x ≠ 201) and must not overlap with the subnet used by port 2. Opening `http://192.168.1.201` in a browser brings up the LiDAR configuration page (to change RPM, IP, etc.).

### Driver Installation and Launch

```bash
sudo apt install ros-noetic-velodyne          # binary install
# or from source (this project's fork, melodic-devel branch, works on Noetic):
cd ~/catkin_ws/src
git clone -b melodic-devel https://github.com/Romindly-Dev/velodyne.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

Launch and verify:

```bash
roslaunch velodyne_pointcloud VLP16_points.launch

rostopic list | grep velodyne
# /velodyne_packets   raw UDP packets
# /velodyne_points    PointCloud2 point cloud
# /scan               single-line LaserScan extracted from the point cloud by the laserscan node

rostopic hz /velodyne_points     # about 10 Hz at 600 rpm
rviz    # set Fixed Frame to velodyne, add PointCloud2 subscribed to /velodyne_points
```

### Key Parameters (args of VLP16_points.launch, overridable on the command line)

| Parameter | Default | Description |
|------|--------|------|
| `device_ip` | empty | LiDAR IP filter; with multiple NICs/LiDARs, explicitly set `192.168.1.201` |
| `port` | 2368 | UDP data port |
| `frame_id` | `velodyne` | Point cloud frame name; the extrinsic TF used when integrating with SLAM is based on this |
| `rpm` | 600.0 | Motor speed (600 rpm = 10 Hz); must match the LiDAR web configuration |
| `min_range` / `max_range` | 0.4 / 130.0 | Range clipping (meters); indoors, lowering max_range to 30 reduces noise |
| `cut_angle` | -0.01 | ≥0 cuts frames at a fixed angle; negative values cut frames by packet count |
| `organize_cloud` | false | Output an organized point cloud (required by some algorithms) |

Example: `roslaunch velodyne_pointcloud VLP16_points.launch device_ip:=192.168.1.201 max_range:=50.0`

## Livox (Horizon / Avia / Mid-40)

### Livox-SDK Installation

`livox_ros_driver` depends on Livox-SDK (a C++ library). The installation steps already appeared in the FAST-LIO section of [Chapter 25 3D LiDAR SLAM](../25_3d_lidar_slam/README.md); as a recap:

```bash
sudo apt install cmake build-essential
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK/build && cmake .. && make -j$(nproc)
sudo make install
```

> The Mid-360 is a new-generation device that uses Livox-SDK2 + `livox_ros_driver2`, which is outside the scope of this repository's driver; this section uses Horizon/Avia/Mid-40 as examples.

### Driver Build and Network

```bash
cd ~/catkin_ws/src
git clone https://github.com/Romindly-Dev/livox_ros_driver.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

Livox LiDARs also use Ethernet (by default they auto-acquire a 65.x address; the host NIC can be set to 192.168.1.50/24 for a direct connection). The driver **identifies devices by broadcast code (the 15-digit serial number on the body sticker)**: fill it into the `broadcast_code` field of `livox_ros_driver/config/livox_lidar_config.json` and set `enable: true`, or pass it as the `bd_list` launch argument.

### Launch and xfer_format

```bash
roslaunch livox_ros_driver livox_lidar_msg.launch      # CustomMsg format, used by FAST-LIO
# or
roslaunch livox_ros_driver livox_lidar.launch          # PointCloud2 format, directly viewable in RViz

rostopic hz /livox/lidar
rostopic echo /livox/imu -n 1      # built-in IMU (Horizon/Avia)
```

`xfer_format` determines the message type of `/livox/lidar` (can be verified in both the README and the launch files):

| xfer_format | Message Type | Purpose |
|-------------|----------|------|
| 0 | PointCloud2 (PointXYZRTL, Livox extended fields) | RViz visualization, general processing (default of `livox_lidar.launch`) |
| 1 | `livox_ros_driver/CustomMsg` (with per-point relative timestamps) | **Required by FAST-LIO/FAST-LIO2** (default of `livox_lidar_msg.launch`) |
| 2 | PointCloud2 (pcl::PointXYZI) | Compatible with standard PCL pipelines |

Other common launch parameters: `publish_freq` (point cloud publish rate, 5/10/20/50 Hz), `multi_topic` (each of multiple LiDARs publishes its own topic), `msg_frame_id` (default `livox_frame`). FAST-LIO relies on the per-point timestamps in CustomMsg for motion distortion compensation, so **always use `livox_lidar_msg.launch` when running FAST-LIO**.

## Integrating with Chapter 25 3D SLAM: Topic Mapping Table

Based on the default configurations of each algorithm in this workspace:

| SLAM Approach | Expected Point Cloud Topic | Expected IMU Topic | Output of This Section's Drivers | Integration |
|-----------|--------------|----------------|--------------|----------|
| A-LOAM | `/velodyne_points` | No IMU used | Velodyne `/velodyne_points` | Works directly |
| LIO-SAM | `points_raw` (`pointCloudTopic` in params.yaml) | `imu_raw` | Velodyne `/velodyne_points` + external IMU | Change topic names in params.yaml, or remap in the launch file |
| FAST-LIO2 (velodyne.yaml) | `/velodyne_points` | `/imu/data` | Velodyne + external IMU | Point cloud works directly; change the IMU topic in the yaml to match the actual driver |
| FAST-LIO2 (avia/horizon.yaml) | `/livox/lidar` (CustomMsg) | `/livox/imu` | `livox_lidar_msg.launch` | Works directly; choose the yaml matching your model |

General notes:

- LIO-SAM / FAST-LIO are sensitive to **LiDAR-IMU extrinsics and time synchronization**; configuration details are covered in the corresponding sections of Chapter 25 — this section only guarantees the driver-side topics are correct.
- The point cloud `frame_id` (`velodyne` / `livox_frame`) must be consistent with the SLAM configuration and the TF tree.
- Persisting the NIC static IP (netplan) and boot-time autostart are covered in [04 udev Rules and Device Management](04_udev_and_device_management.md).

## Common Issues

**1. Cannot `ping` the LiDAR**: the NIC is not up (`ip link set ... up`), the static IP subnet is wrong, or the Interface Box is not powered (the VLP-16 needs a 12V adapter).

**2. Topic exists but `rostopic hz` shows no output**: with multiple NICs, the UDP packets went to another interface — explicitly set `device_ip`, and confirm the firewall is not blocking port 2368 (`sudo ufw status`).

**3. The Livox driver keeps printing `wait for lidar` after starting**: the broadcast_code is wrong or `enable` is false; check the body sticker and inspect the json.

**4. FAST-LIO reports a point cloud message type mismatch**: you used `livox_lidar.launch` (PointCloud2) instead of `livox_lidar_msg.launch` (CustomMsg).

---

Previous: [01 RPLIDAR](01_rplidar.md) | Next: [03 RealSense](03_realsense.md)
