# 01 Real-Machine Deployment Checklist

> 中文版 / Chinese: [01_实机部署清单.md](../../50_部署运维/01_实机部署清单.md)

## Goal

Starting from a blank x86_64 edge unit, follow the checklist through system settings → ROS installation → time synchronization → workspace deployment → network configuration → acceptance tests, ending up with a machine whose "environment is reproducible and is ready to hand over to Section 02 for autostart configuration".

## Background

The biggest difference between deployment and development is that **non-reproducible manual operations are liabilities**: three months from now, when a second machine needs to replicate today's environment, anything "tweaked by hand at the time" becomes a trap. That is why every step in this section is given as commands, and we recommend consolidating them into your own deployment script. Another piece of context: ROS Noetic reached EOL in May 2025—the apt repository is still online but no longer updated. Bare-metal installation still works, but for long-term maintenance [03 Docker Deployment](03_docker_deployment.md) is recommended.

## Step 1: Ubuntu 20.04 Base Setup

Install Ubuntu 20.04.x (Desktop or Server both work; for unattended machines, Server/minimal installation is recommended—it saves memory and reduces background interference). Do three things right after installation:

```bash
# 1. Disable automatic sleep/suspend (a sleeping field machine = a robot gone offline)
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# 2. Disable automatic updates (avoid unattended-upgrades pulling packages, holding the apt lock, or even rebooting in the middle of the night)
sudo systemctl disable --now unattended-upgrades apt-daily.timer apt-daily-upgrade.timer

# 3. On machines with a desktop, also disable idle suspend at the graphical layer
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing' 2>/dev/null || true
```

Verify: `systemctl status sleep.target` should show `Loaded: masked`.

## Step 2: Install ROS Noetic (Including the Expired apt Key Fix)

See [Chapter 00](../00_setup/01_install_ubuntu2004_ros_noetic.md) for the full installation procedure. Edge units usually only need `ros-noetic-ros-base` (without GUI packages such as rviz/rqt), then use rosdep with `ros1.repos` to fill in the remaining dependencies.

### Must-Read: Fixing the EXPKEYSIG Error

The GPG signing key of the official ROS apt repository was rotated in mid-2025. On systems installed from old images, or with sources configured following outdated tutorials, `apt update` reports:

```
Err:5 http://packages.ros.org/ros/ubuntu focal InRelease
  The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
```

The fix is to re-download the latest ros.key and overwrite the old keyring:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

Also make sure the source entry references that keyring (the modern syntax):

```bash
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros-latest.list
```

> Machines using the Tsinghua mirror may hit the same problem (the mirror syncs the official signatures); the fix is identical. We recommend adding this step to your deployment script and running it uniformly on both new and old machines.

## Step 3: Time Synchronization (chrony)

In multi-machine deployments, TF and sensor messages all rely on timestamp alignment: if two machines differ by a few hundred milliseconds, TF reports `extrapolation into the past` and fusion and mapping fall apart. Timestamps stamped by sensors (e.g. Velodyne) also depend on the host clock. Use chrony instead of the default systemd-timesyncd—it converges faster and supports machines serving as time sources for each other on a local network:

```bash
sudo apt install -y chrony
```

Machines with internet access can use the default configuration; **on an offline production line**, let the main control machine act as the time server. Edit `/etc/chrony/chrony.conf`:

```bash
# —— Main control machine (time server): append ——
allow 192.168.1.0/24
local stratum 8          # serve local time to others even without internet access

# —— All other machines: comment out the pool lines, point to the main control machine ——
server 192.168.1.10 iburst
```

```bash
sudo systemctl restart chrony
chronyc tracking          # check the System time offset
chronyc sources -v        # on client machines, the main control machine should appear as the selected source (^*)
```

Acceptance criterion: the `System time` offset in `chronyc tracking` is < 10 ms.

## Step 4: Deploy the Workspace (Two Routes)

| Route | Approach | Pros | Cons | Suitable For |
| --- | --- | --- | --- | --- |
| A. Source deployment | `vcs import` sources on the machine + `catkin_make` | Same workflow as the development machine; code can be modified and tuned on the machine | Depends on internet access/build toolchain; slow builds; on-machine code easily gets messed up in the field | Pilot prototypes; on-site debugging needed |
| B. Copy build artifacts | Build on the development machine, copy the entire `install/` directory over | No internet or compiler needed; fast deployment; read-only artifacts | Development and target machines must have identical OS/dependencies; missing apt dependencies cause missing libraries | Batch production-line deployment |

**Route A** (same as the [Chapter 00 workspace section](../00_setup/02_workspace_and_code.md)):

```bash
mkdir -p ~/ws_romindly/src && cd ~/ws_romindly
git clone https://github.com/Romindly-Dev/romindly_ros1_workspace.git src/romindly_ros1_workspace
vcs import src < src/romindly_ros1_workspace/ros1.repos
rosdep install --from-paths src --ignore-src -r -y
catkin_make && catkin_make install
```

**Route B**: after `catkin_make install` on the development machine:

```bash
rsync -avz ~/ws_romindly/install/ iot@192.168.1.10:~/ws_romindly/install/
# On the target machine, just source the install space to run
source ~/ws_romindly/install/setup.bash
```

### Why Production Lines Should Use the install Space

Many files in `devel/` are **symlinks back to the src/ sources**—copying it elsewhere turns them into broken links. The `install/` directory generated by `catkin_make install` is a self-contained deliverable (binaries + launch + config + setup.bash) that does not depend on src existing. Shipping only the install space to the production line means: no source code on site to mess with, the directory can be verified as a whole (md5), and rollback = swapping directories. Note that packages like `robot_bringup` must correctly `install()` their launch/config in CMakeLists for them to end up in the install space.

## Step 5: Network Configuration

### netplan Static IP

Edge units must have fixed IPs (same syntax as the LiDAR Ethernet port in [Chapter 45](../45_sensor_drivers/04_udev_and_device_management.md)). Edit `/etc/netplan/01-robot.yaml`:

```yaml
network:
  version: 2
  ethernets:
    enp1s0:
      dhcp4: no
      addresses: [192.168.1.10/24]
      routes: [{to: default, via: 192.168.1.1}]
      nameservers: {addresses: [223.5.5.5, 8.8.8.8]}
```

```bash
sudo netplan apply && ip addr show enp1s0
```

### Multi-Machine ROS Configuration

Suppose the main control machine (running roscore) is 192.168.1.10 and the operator station is 192.168.1.20. Set on each machine respectively:

```bash
# Main control machine
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.10
# Operator station
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.20
```

**The biggest trap: hostname resolution.** When a ROS1 node registers, it reports its own address, which defaults to its hostname; if the other side cannot resolve that hostname, the symptom is that `rostopic list` works (it only queries the master) while `rostopic echo` receives no data (the peer-to-peer connection fails). Two solutions, pick either one, and it **must be set on every machine**:

- Directly `export ROS_IP=<this machine's IP>` (recommended—bypasses resolution);
- Or register `IP hostname` pairs for each other in `/etc/hosts` on all machines, then use `ROS_HOSTNAME`.

For single-machine deployments, keep `localhost`—see the comments in [ros_env.sh](https://github.com/Romindly-Dev/romindly_robot_bringup/blob/main/robot_bringup/scripts/ros_env.sh).

## Step 6: Acceptance Test Checklist

Run through each item after deployment; each one maps to a capability taught in an earlier chapter:

| # | Command | Expected | Chapter |
| --- | --- | --- | --- |
| 1 | `rosversion -d` | `noetic` | 00 |
| 2 | `ls -l /dev/rplidar /dev/base_serial` | Symlinks exist | 45 |
| 3 | `roslaunch robot_bringup robot.launch` | No red ERROR output | 45 |
| 4 | `rostopic hz /scan` | Close to the LiDAR's nominal rate (e.g. 10 Hz) | 45 |
| 5 | `rosrun tf2_tools view_frames.py` | TF tree connected: odom→base_link→laser | 10 |
| 6 | `rostopic echo /odom -n1` | Odometry output present | 10 |
| 7 | `rostopic echo /scan -n1` on the operator station | Data received across machines (verifies ROS_IP) | This section |
| 8 | `chronyc tracking` | Offset < 10 ms | This section |
| 9 | Send a navigation goal | Robot reaches the goal | 40 |
| 10 | After a power cycle, items 1–8 recover automatically | See [02 systemd Autostart](02_systemd_autostart.md) | 50 |

## Common Issues

- **`apt update` reports EXPKEYSIG**: see Step 2—reinstall ros.key to overwrite `/usr/share/keyrings/ros-archive-keyring.gpg`.
- **`rostopic list` works but `echo` shows no data**: hostname resolution problem; check `ROS_IP`/`/etc/hosts` on both ends.
- **Nodes missing `.so` libraries after copying the install space**: the target machine is missing apt dependencies; use `ldd` to find which library is missing, or run `rosdep install` once on the target machine.
- **TF reports extrapolation / lookup time out of range**: check `chronyc sources` first—in multi-machine setups the clocks are not synchronized.
- **Machine goes offline overnight**: automatic sleep was not fully disabled; go back to Step 1 and check whether `sleep.target` is masked.
