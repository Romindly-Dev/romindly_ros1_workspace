# 04 udev Rules and Device Management

> 中文版 / Chinese: [04_udev规则与设备管理.md](../../45_传感器驱动/04_udev规则与设备管理.md)

## Goals

- Understand why serial device names drift, and use udev rules to create pinned symlinks `/dev/rplidar`, `/dev/imu`, and `/dev/base_serial` for the RPLIDAR, IMU, and base serial ports.
- Use netplan to configure a persistent static IP on the Ethernet port for the Velodyne/Livox scenario.
- Prepare the "device ready" precondition checks for boot-time autostart in [Chapter 50 Deployment and Operations](../50_deployment/README.md).

## Why Device Names Drift

Linux assigns `/dev/ttyUSB0`, `/dev/ttyUSB1`, ... to USB serial devices by **enumeration order** — whichever device the kernel discovers first gets the lower number. Plug-in order, power-up timing, and hub ports all change the enumeration order: today the LiDAR is `ttyUSB0` and the IMU is `ttyUSB1`; after a reboot they may swap. Hard-coding `ttyUSB0` in a launch file results in "the LiDAR driver opening the IMU's serial port" — random failures that are hard to debug.

The solution: udev matches on stable attributes such as **VID/PID/serial number** when a device is plugged in and creates a symlink with a fixed name; all launch files then use the symlink names. The `sensors.launch` and `config/base.yaml` of [robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) use exactly `/dev/rplidar` and `/dev/base_serial`.

## Step 1: Query Device Attributes

After plugging in the device, query its identity with `udevadm info`:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial" | head -6
```

Typical output (the RPLIDAR's CP210x adapter board):

```
ATTRS{idVendor}=="10c4"
ATTRS{idProduct}=="ea60"
ATTRS{serial}=="0001"
```

Common chips: CP210x (`10c4:ea60`, commonly used by Slamtec LiDARs), CH340 (`1a86:7523`, many domestic IMU/base boards), FTDI (`0403:6001`). Plug in each device one by one and record the triples; **devices with the same chip model must be distinguished by `serial`**.

## Step 2: Write the Rules File

Create `/etc/udev/rules.d/99-robot-devices.rules` (the numeric prefix determines ordering; 99 ensures it runs after the system default rules):

```bash
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```

```
# RPLIDAR (CP210x), symlink /dev/rplidar, also open up permissions (can replace the dialout group approach)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE="0666", SYMLINK+="rplidar"

# IMU (CH340), symlink /dev/imu
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE="0666", SYMLINK+="imu"

# base serial port (FTDI), symlink /dev/base_serial — matches the port in robot_bringup/config/base.yaml
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", \
  MODE="0666", SYMLINK+="base_serial"
```

> Use the VID/PID values you actually measured with `udevadm info`; the above are only examples of common chips.

### Conflicts Between Identical Devices: Distinguish by serial

If the IMU and the base both use the same CH340 chip, two rules matching only on VID/PID will both match. Add `ATTRS{serial}` (or, when there is no serial number, a physical port path such as `KERNELS=="1-2"`) for precise distinction:

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="IMU0001", SYMLINK+="imu", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="BASE001", SYMLINK+="base_serial", MODE="0666"
```

## Step 3: Load and Verify

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger            # or replug the device
ls -l /dev/rplidar /dev/imu /dev/base_serial
# lrwxrwxrwx 1 root root 7 ... /dev/rplidar -> ttyUSB0
```

From then on, always use the symlink names in launch files, e.g. the LiDAR from [Section 01](01_rplidar.md):

```bash
roslaunch rplidar_ros rplidar_a1.launch serial_port:=/dev/rplidar
# robot_bringup's sensors.launch already uses /dev/rplidar by default
```

## Ethernet Devices: netplan Static IP (Velodyne Scenario)

The `ip addr add` in [Section 02](02_velodyne_and_livox.md) is temporary and lost on reboot. Ubuntu 20.04 persists it with netplan. Edit `/etc/netplan/01-lidar.yaml` (interface names per the actual output of `ip link`):

```yaml
network:
  version: 2
  ethernets:
    enp1s0:                 # port 1: directly connected to Velodyne (default 192.168.1.201)
      addresses: [192.168.1.100/24]
      dhcp4: false
    enp2s0:                 # port 2: internet, keep DHCP
      dhcp4: true
```

```bash
sudo netplan apply
ping 192.168.1.201
```

Note: **do not configure a gateway** on the LiDAR port, otherwise it may hijack the default route and cut off the internet port; the two ports' subnets must not overlap.

## Device Readiness Checks Before Boot-Time Autostart

[Chapter 50](../50_deployment/README.md) uses `ros-robot.service` for boot-time autostart. When the system service starts, USB enumeration may not have finished, and directly starting the launch will fail because the device does not exist. Two approaches:

**Approach A: udev TAG + systemd device unit (recommended).** Tag the device for systemd in the rule:

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE="0666", SYMLINK+="rplidar", TAG+="systemd", ENV{SYSTEMD_ALIAS}="/dev/rplidar"
```

Then add the dependency to the `[Unit]` section of `ros-robot.service` (`dev-rplidar.device` is systemd's unit name for `/dev/rplidar`):

```ini
[Unit]
Description=ROS1 Robot Bringup
After=network-online.target dev-rplidar.device
Wants=network-online.target
Requires=dev-rplidar.device
```

This makes the service wait until the LiDAR device is ready before starting, and prevents the service from running blindly when the device is unplugged.

**Approach B: polling inside the startup script.** Add wait logic at the top of `ros_env.sh` — crude but universal:

```bash
for i in $(seq 1 30); do
  [ -e /dev/rplidar ] && break
  sleep 1
done
```

Combined with `Restart=on-failure` (already included in the service template), the service self-heals even if the device is occasionally not ready.

## Common Issues

**1. Rules written but the symlink does not appear**: after `udevadm control --reload-rules` you must **replug the device** or run `udevadm trigger`; then use `udevadm test $(udevadm info -q path -n /dev/ttyUSB0) 2>&1 | grep -i symlink` to check whether the rule matched.

**2. The symlink points to the wrong device**: two devices share the same VID/PID and no `serial` condition was added; distinguish them by serial number or physical port path as described above.

**3. `ATTRS{serial}` cannot be found**: cheap CH340 chips often have no serial number; use `KERNELS=="1-2"` instead (the KERNELS value from `udevadm info -a`, binding to the physical USB port — at the cost that the device cannot be plugged into a different port).

**4. Internet connection lost after netplan apply**: the LiDAR port was configured with `gateway4`/`routes` and stole the default route; delete it.

---

Previous: [03 RealSense](03_realsense.md) | Back to [Chapter Index](README.md)
