# 04 Performance Optimization and Operations

> 中文版 / Chinese: [04_性能优化与运维.md](../../50_部署运维/04_性能优化与运维.md)

## Goals

- Learn to use htop/iotop + `rostopic hz/delay` to determine whether the bottleneck is compute or transport, and reduce load per module using the lookup table.
- Master CPU pinning, log management, remote operations (remote rviz, rosbag black box), and the upgrade strategy under EOL.

## Background

An edge unit's compute is a fixed budget: with 3D SLAM, navigation, and sensor drivers running simultaneously, the direct symptoms of a saturated CPU are dropped topic rates, TF timeouts, and jerky navigation. The first step of operations is not tuning parameters but **locating the bottleneck**—first distinguish "cannot compute fast enough" from "cannot transmit fast enough", then apply the right remedy.

## Step 1: Resource Monitoring and Bottleneck Identification

```bash
sudo apt install -y htop iotop
htop          # view CPU per core; F6 to sort by CPU% and find the big consumers (typically: SLAM nodes, move_base, rviz)
sudo iotop -o # show only processes doing IO; heavy disk writers like rosbag record / rosout show up here
df -h /       # disk usage level; a must-check for long-running machines
```

Two measuring sticks on the ROS side:

```bash
rostopic hz /scan            # actual rate vs nominal rate
rostopic delay /scan         # latency from the message header timestamp to reception (messages must carry a header)
```

How to interpret:

| Symptom | Conclusion | Direction |
| --- | --- | --- |
| Publisher-side hz already low, one CPU core saturated | **Compute bottleneck**: the node cannot keep up | Reduce load (table below), pin cores |
| Publisher-side hz normal, subscriber-side hz low / delay large | **Transport bottleneck**: bandwidth or cross-machine network | Downsample, compress transport, check WiFi |
| hz normal, periodic delay spikes | Scheduling jitter or unsynchronized clocks | Pin cores; for multi-machine setups check chrony first ([Section 01](01_deployment_checklist.md)) |

For images/point clouds across machines, also use `rostopic bw <topic>` to estimate bandwidth—on WiFi a point cloud easily saturates the link.

## Step 2: Per-Module Load-Reduction Summary Table

Parameters taught in earlier chapters, recapped here from the "operational load-shedding" angle:

| Module | Load-reduction technique | Parameters/method | Details |
| --- | --- | --- | --- |
| costmap | Lower resolution, lower update rate, shrink the local window | `resolution: 0.05→0.1`, `update_frequency: 5→2`, `width/height` | [Chapter 40 costmap tuning](../40_navigation/02_costmap_tuning.md) |
| gmapping | Fewer particles, larger map update interval | `particles: 30→10`, increase `map_update_interval` | [Chapter 20 gmapping](../20_2d_slam/02_gmapping.md) |
| 3D SLAM | Larger voxel filters, lower output rate | Increase FAST-LIO `filter_size_surf/map`; same idea for LIO-SAM | [Chapter 25](../25_3d_lidar_slam/README.md) |
| Point cloud | Downsample upstream before feeding the algorithm | `pcl_ros` VoxelGrid nodelet, `leaf_size: 0.1` | [Chapter 25](../25_3d_lidar_slam/01_overview_and_setup.md) |
| Visualization | **Do not run rviz on the robot** | A single rviz process can eat 1–2 cores: switch to remote rviz (below) or web solutions (rosboard/foxglove) | This section |
| Local planner | Fewer samples | DWA `vx_samples`, TEB horizon/optimization steps | [Chapter 40](../40_navigation/03_dwa_and_teb.md) |

Principle: first reduce "what you watch" (visualization, debug topics), then "what you build" (mapping precision), and only last touch "what drives" (do not lightly reduce control frequency).

## Step 3: CPU Pinning and Real-Time Considerations

Pin critical nodes to dedicated cores to keep the big consumers from squeezing each other:

```bash
# Look up the PID, then pin (pin the SLAM node to cores 2 and 3)
taskset -cp 2,3 $(pgrep -f fastlio_mapping)

# Or bake it into the launch file with launch-prefix
<node pkg="..." type="..." name="slam" launch-prefix="taskset -c 2,3"/>
```

For system-level isolation, use cgroup/cpuset to confine system housekeeping to cores 0 and 1, leaving cores 2 and up for ROS—the easiest way is systemd's `CPUAffinity=` (added to the `[Service]` section of the [Section 02](02_systemd_autostart.md) service).

Use real-time scheduling `chrt -r` (SCHED_RR) **with caution**: ROS1 nodes were not written for hard real-time, and elevating a CPU-hungry node to real-time priority can starve system processes until the machine goes offline. For a typical robot, core pinning + `nice -n -5` is enough.

## Step 4: Log Operations

Without log management, a long-running machine is guaranteed to fill its disk. Three fronts:

```bash
# 1. journald cap (systemd service logs, already mentioned in Section 02)
sudo sed -i 's/^#\?SystemMaxUse=.*/SystemMaxUse=500M/' /etc/systemd/journald.conf
sudo systemctl restart systemd-journald

# 2. rosout file logs: every roslaunch creates a new directory under ~/.ros/log and never cleans up
rosclean check    # check current usage
rosclean purge -y # purge manually

# 3. Periodic cleanup via cron (keep 7 days), add with crontab -e:
0 3 * * * find /home/iot/.ros/log -maxdepth 1 -mtime +7 -exec rm -rf {} +
```

> In containers running as root, logs are under `/root/.ros/log`; [Section 03](03_docker_deployment.md) already mounts it to a host volume, so include it in the cron job above as well. You can also lower the log level for noisy nodes in the launch file: `<env name="ROSCONSOLE_MIN_SEVERITY" value="WARN"/>`.

## Step 5: Remote Operations

**Remote rviz**: the robot runs headless—do not install or run rviz on it. Set `ROS_MASTER_URI` on the operator station to point to the robot ([Section 01](01_deployment_checklist.md) multi-machine configuration), then run rviz locally so the rendering cost is paid on the operator station. When the network is not fully transparent, fall back to ssh port forwarding:

```bash
ssh -L 11311:localhost:11311 iot@192.168.1.10   # forward the master port
# Note: ROS1 nodes also use random ports, so forwarding alone is incomplete; the proper solution is full network connectivity + ROS_IP—use forwarding only to inspect the master in emergencies
```

**rosbag black box**: record periodic snapshots of key topics, and replay them after an incident to diagnose it (for replay, see the [Chapter 10 rosbag section](../10_ros1_basics/08_rosbag_and_debugging.md)):

```bash
# Record 60 seconds of key topics every hour, driven by cron; --split --max-splits controls total volume
0 * * * * bash -lc 'source /opt/ros/noetic/setup.bash && rosbag record -O /home/iot/robot_data/bags/snap_$(date +\%H).bag --duration=60 /scan /odom /tf /rosout'
```

Naming by the hour automatically overwrites the previous day's file for the same time slot, keeping disk usage constant. For continuous recording, switch to `rosbag record --split --size=1024 --max-splits=10` to keep the most recent 10 GB in a ring.

**Crash self-healing**: node crash-restarts are already covered by `Restart=on-failure` in [Section 02](02_systemd_autostart.md); on the operations side, just periodically check the restart count in journalctl: if `journalctl -u ros-robot | grep -c "Started ROS1"` grows abnormally, it is time to investigate.

## Step 6: Firmware/System Upgrade Strategy (the EOL Reality)

Noetic and Ubuntu 20.04 have both entered their EOL cycle, so the upgrade strategy is a single word: **freeze**.

- Bare-metal machines: no `apt upgrade` (automatic updates were already disabled in [Section 01](01_deployment_checklist.md)). After EOL the benefit of upgrading approaches zero while the risk remains ever-present (once a dependency is disturbed, it may never install back the same way).
- All functional changes go through **Docker image versioning** ([Section 03](03_docker_deployment.md)): change the code → build `romindly/ros1-noetic:1.1` → verify on the bench → distribute to the production line via save/load → roll back to `:1.0` if there is a problem. The image tag is the release version number, and the host is never touched.
- For internet-connected devices that genuinely need security patches, evaluate Ubuntu Pro (20.04 ESM extended maintenance) to cover the OS layer, while the ROS layer remains frozen.

## Common Issues

- **CPU not high but navigation stutters**: check `rostopic delay /tf` and chrony—unsynchronized clocks across machines are often misdiagnosed as a performance problem.
- **The robot lags as soon as rviz opens**: rviz is running on the robot itself—switch to remote rviz; also, subscribing to point cloud topics costs the robot bandwidth/CPU on its own, so remember to close them after debugging.
- **Disk fills up after a few weeks**: three suspects, checked in order—`~/.ros/log` (rosclean), the journal (SystemMaxUse), and the rosbag directory (ring-overwrite strategy).
- **More lag after pinning cores**: too few cores were allocated—a node's internal threads (e.g. move_base) got squeezed onto two cores. In `htop`, show threads (H key) to confirm the thread count before allocating.
- **Want to upgrade a ROS package to fix a bug**: there is no newer version in the EOL repository; build the package from source as an overlay in the workspace, and bake the change into the next Docker image version.
