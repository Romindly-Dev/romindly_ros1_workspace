# 01 RPLIDAR

> 中文版 / Chinese: [01_RPLIDAR接入.md](../../45_传感器驱动/01_RPLIDAR接入.md)

## Goals

- Connect a Slamtec RPLIDAR (A1/A2/A3/S/C series) to Romindly Mind via USB serial, build `rplidar_ros`, and publish `/scan`.
- Verify the data with `rostopic hz` and RViz, and finally feed it into gmapping / slam_toolbox mapping from Chapter 20.

## Hardware Connection

1. The RPLIDAR connects to a Type-C port on Romindly Mind through the bundled USB adapter board (CP210x chip) — a Type-C to USB-A hub can be used in between.
2. The A series can be powered directly over USB; the S series draws more power, so a **USB hub with independent power supply** is recommended to avoid unstable motor speed caused by insufficient voltage.
3. After plugging in, confirm that the device node appears:

```bash
ls -l /dev/ttyUSB*
dmesg | grep -i cp210x        # you should see "cp210x converter now attached to ttyUSB0"
```

### Serial Port Permissions

A regular user has no read/write access to `/dev/ttyUSB0` by default. The recommended approach is to join the `dialout` group (configure once, effective permanently):

```bash
sudo usermod -aG dialout $USER
# takes effect after logging out and back in; confirm with the groups command
```

Temporary workaround (lost after reboot/replug, for emergencies only):

```bash
sudo chmod 666 /dev/ttyUSB0
```

> When multiple serial devices coexist, `/dev/ttyUSB0` will drift. For production deployment, pin it to `/dev/rplidar` following [04 udev Rules](04_udev_and_device_management.md).

## Driver Installation

Use the organization's fork of the official Slamtec driver (master branch):

```bash
cd ~/catkin_ws/src
git clone https://github.com/Romindly-Dev/rplidar_ros.git
cd ~/catkin_ws
catkin_make            # or catkin build
source devel/setup.bash
```

## Launch and Verification

The driver provides separate launch files per model (`rplidar_ros/launch/`). The core node is always `rplidarNode`; the main difference is the baud rate:

```bash
# choose one according to your model
roslaunch rplidar_ros rplidar_a1.launch      # A1 / A2M8
roslaunch rplidar_ros rplidar_a3.launch      # A3
roslaunch rplidar_ros rplidar_s2.launch      # S2
```

A normal startup log prints the SDK version, serial number, and `current scan mode`. Then verify:

```bash
rostopic hz /scan
# A1 about 5.5~10 Hz, S/C series about 10 Hz; a stable rate means it is working

rostopic echo /scan -n 1 | head -20
# check header.frame_id: "laser" and that the ranges array contains valid distance values
```

Viewing in RViz (the driver ships view_ launch files with an rviz config):

```bash
roslaunch rplidar_ros view_rplidar_a1.launch    # substitute your model accordingly
```

Or start `rviz` manually: set Fixed Frame to `laser`, add a LaserScan display subscribed to `/scan`.

## Parameter Table (rplidarNode)

The following parameters are taken from the launch files in this repository; refer to the actual files as the source of truth:

| Parameter | Type | Description | Default/Typical Value |
|------|------|------|-------------|
| `serial_port` | string | Serial device name | `/dev/ttyUSB0` (change to `/dev/rplidar` for deployment) |
| `serial_baudrate` | int | Baud rate, **varies by model**, see the table below | 115200 |
| `frame_id` | string | Coordinate frame name for scan data | `laser` |
| `inverted` | bool | Whether the LiDAR is mounted upside down | `false` |
| `angle_compensate` | bool | Angle compensation for a uniform number of points per frame | `true` |
| `scan_mode` | string | Scan mode (the A3 launch uses `Sensitivity`, C1 uses `Standard`) | empty = firmware default |
| `scan_frequency` | double | Scan frequency in Hz (provided by the launch files of newer models such as S2/C1) | `10.0` |

Baud rate per model (see the launch files in the repository as the source of truth):

| Model | Launch File | serial_baudrate |
|------|-------------|-----------------|
| A1 / A2M7 / A2M8 | `rplidar_a1.launch`, etc. | 115200 |
| A2M12 / A3 / S1 | `rplidar_a2m12.launch` / `rplidar_a3.launch` / `rplidar_s1.launch` | 256000 |
| C1 | `rplidar_c1.launch` | 460800 |
| S2 / S3 / S2E | `rplidar_s2.launch`, etc. | 1000000 |

> The typical symptom of a baud rate mismatch is the startup error `Error, operation time out` — first check that the model and launch file correspond.

## Integrating with Chapter 20 Mapping

gmapping / slam_toolbox need `/scan` plus the `base_link → laser` TF. A minimal combined launch (compare with [Chapter 20 gmapping in Practice](../20_2d_slam/02_gmapping.md)):

```xml
<launch>
  <!-- LiDAR driver -->
  <include file="$(find rplidar_ros)/launch/rplidar_a1.launch"/>

  <!-- base_link -> laser static TF; adjust the translation to the actual mounting position -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="laser_tf"
        args="0.10 0 0.18 0 0 0 base_link laser"/>

  <!-- gmapping (also requires the base to provide the odom->base_link TF) -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <remap from="scan" to="/scan"/>
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="odom"/>
  </node>
</launch>
```

Key points:

- `frame_id` (default `laser`) must match the child frame of the static TF; otherwise RViz reports missing TF and gmapping produces no output.
- gmapping also depends on the odometry TF (`odom → base_link`). With only a LiDAR and no base, first verify with [Chapter 20 hector_slam](../20_2d_slam/03_hector_slam.md).
- [robot_bringup's sensors.launch](https://github.com/Romindly-Dev/romindly_robot_bringup) already bundles this section's driver + static TF combination; for real-robot deployment just run `roslaunch robot_bringup sensors.launch use_rplidar:=true`.
- Integrating slam_toolbox works the same way, see [Chapter 20 slam_toolbox in Practice](../20_2d_slam/04_slam_toolbox.md).

## Common Issues

**1. `Error, operation time out. RESULT_OPERATION_TIMEOUT`**
- Baud rate does not match the model (most common); switch launch files according to the baud rate table above;
- The serial port is occupied by another process (investigate with `sudo lsof /dev/ttyUSB0`);
- The USB cable is too long or of poor quality; use a short cable connected directly.

**2. Failed to open serial port / Permission denied**
- Not in the `dialout` group or not re-logged in yet; as an emergency fix, `sudo chmod 666 /dev/ttyUSB0`.

**3. Motor not spinning or speed fluctuating, unstable `/scan` rate**
- Insufficient USB power, especially noticeable on the S series; switch to a hub with independent power supply or a native USB3 port on the host.

**4. Device name is not ttyUSB0**
- With multiple serial devices present, numbering drifts with plug-in order; temporarily specify `serial_port:=/dev/ttyUSB1`, and see [04 udev Rules](04_udev_and_device_management.md) for the long-term solution.

**5. Point cloud direction is flipped in RViz**
- Set `inverted:=true` when the LiDAR is mounted upside down; correct mounting orientation offsets via the yaw of the static TF.

---

Next: [02 Velodyne and Livox](02_velodyne_and_livox.md)
