# 02 systemd Autostart on Boot

> 中文版 / Chinese: [02_systemd开机自启.md](../../50_部署运维/02_systemd开机自启.md)

## Goals

- Understand the [ros_env.sh](https://github.com/Romindly-Dev/romindly_robot_bringup/blob/main/robot_bringup/scripts/ros_env.sh) and [ros-robot.service](https://github.com/Romindly-Dev/romindly_robot_bringup/blob/main/systemd/ros-robot.service) provided by [robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup); complete installation, enablement, and log-based troubleshooting.
- Implement three hands-on improvements: waiting for devices to be ready, a roscore/application dual-service architecture, and graceful shutdown.

## Background: Why systemd

The requirements for a delivered machine are "starts on power-up, gets back on its feet after a crash, and logs are always available somewhere". systemd is the service manager that ships with Ubuntu and satisfies each requirement: it starts services in dependency order at boot, `Restart=` restarts them automatically, and `journalctl` collects logs in one place. Compared with makeshift approaches like `rc.local` or crontab `@reboot`, it can express dependency relationships such as "wait for the network to be ready before starting" and "if A dies, restart B along with it".

## Line-by-Line Walkthrough

### ros_env.sh — the Service Entry Script

```bash
#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/iot/workspace/ws_romindly/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

exec roslaunch robot_bringup robot.launch --wait
```

- **Why source explicitly?** When systemd starts a process, the environment is almost empty—it **does not read `.bashrc`** (that is configuration for interactive shells). You can `roslaunch` in a terminal because `.bashrc` sourced setup.bash for you; inside a service you must do it yourself. Likewise, variables such as `ROS_MASTER_URI` must be explicitly exported in the script. In multi-machine deployments, change the two localhost entries to actual IPs (see the network section of [Section 01](01_deployment_checklist.md)).
- **`exec`**: the roslaunch process **replaces** the bash process, so systemd supervises roslaunch directly—the PID matches, signals (stop/restart) reach it directly, and you never end up with "bash dies and roslaunch becomes an orphan".
- **`--wait`**: roslaunch waits for a roscore to appear instead of starting one itself, preparing for the dual-service architecture below (in single-service mode it also runs without this flag—roslaunch starts a master on its own).
- Note the path is `devel/setup.bash`; on machines deployed via Route B of Section 01 with an install space, change it to `install/setup.bash`.

### ros-robot.service — the Service Unit

```ini
[Unit]
Description=ROS1 Robot Bringup
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=iot
ExecStart=/home/iot/workspace/ws_romindly/src/romindly_robot_bringup/robot_bringup/scripts/ros_env.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

| Line | Purpose | Notes |
| --- | --- | --- |
| `After=network-online.target` | Ordering: start only after "network online" | Defines order only, not dependency; must be paired with Wants |
| `Wants=network-online.target` | Weak dependency: pulls that target into the boot set | Strongly needed for multi-machine deployments; recommended even for pure localhost |
| `Type=simple` | The ExecStart process itself is the main process | Works with the `exec` in the script |
| `User=iot` | Run as user iot | No root needed: serial-port permissions go through the dialout group/udev; the log directory is `/home/iot/.ros` |
| `ExecStart=` | Absolute path to the entry script | Always use absolute paths in systemd |
| `Restart=on-failure` | Automatically restart on non-zero exit | Crash self-healing; a normal stop via `systemctl stop` does not trigger it |
| `RestartSec=5` | Wait 5 seconds before restarting | Gives devices/network a breather and avoids restart storms flooding the logs |
| `WantedBy=multi-user.target` | After enable, attaches to the multi-user run level | Starts even on a desktop-less Server |

## Installation, Enablement, and Logs

```bash
cd ~/ws_romindly/src/romindly_robot_bringup
sudo cp systemd/ros-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ros-robot.service   # enable = autostart on boot + start immediately

systemctl status ros-robot        # Active: active (running) means success
journalctl -u ros-robot -f        # live log (all roslaunch output is here)
journalctl -u ros-robot -b        # all logs since this boot
journalctl -u ros-robot --since "10 min ago" -p err   # only recent errors
```

After modifying the `.service` file you must run `daemon-reload` and then `restart` for changes to take effect. Acceptance: `sudo reboot`, then after the reboot run items 3–6 of the [Section 01 acceptance checklist](01_deployment_checklist.md).

## Improvement 1: Wait for Devices to Be Ready

Network ready does not mean the LiDAR is ready. When udev is slow to process, the driver reports "cannot open /dev/rplidar" and relies on Restart to retry—it works, but it is not elegant. systemd automatically generates device units for devices, with paths escaped `/` → `-`; depend on them directly (echoing the [Chapter 45 udev section](../45_sensor_drivers/04_udev_and_device_management.md)):

```ini
[Unit]
After=network-online.target dev-rplidar.device
Wants=network-online.target
Requires=dev-rplidar.device
```

`Requires=` means: do not start if the device is absent, and stop the service if the device disappears. For multiple devices, list more of them (`dev-base_serial.device`, etc.). Verify that the device unit exists: `systemctl status dev-rplidar.device`.

## Improvement 2: A Separate roscore Service (Dual-Service Architecture)

In single-service mode, a roslaunch crash-restart takes the master down with it: everything on the parameter server (map, calibration parameters) is lost, and nodes on other machines all have to reconnect. Splitting roscore out as an independently supervised service is more robust. Create `/etc/systemd/system/ros-core.service`:

```ini
[Unit]
Description=ROS1 Master (roscore)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=iot
ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash && exec roscore"
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Then make ros-robot.service depend on it:

```ini
[Unit]
After=network-online.target ros-core.service dev-rplidar.device
Requires=ros-core.service
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now ros-core ros-robot
```

The `--wait` in `ros_env.sh` now earns its keep: roslaunch waits for the master from ros-core to be ready instead of starting its own. Result: when the application crashes, only the application restarts—the master and its parameters stay put.

## Improvement 3: Graceful Shutdown

roslaunch performs its normal shutdown sequence only on **SIGINT** (equivalent to Ctrl+C): it notifies each node to shut down, and nodes run their cleanup (the base sends a stop command, serial ports close, logs are flushed). systemd, however, sends SIGTERM by default, and roslaunch's handling of it is not as clean as SIGINT. Add to the `[Service]` section:

```ini
KillSignal=SIGINT
TimeoutStopSec=20
```

`TimeoutStopSec` gives nodes 20 seconds to wrap up before SIGKILL forcibly terminates them. For scenarios where "the base must receive a stop command when the service stops", these two lines are a safety item, not a cosmetic one.

## Common Issues

- **Starts too early at boot, nodes cannot reach the network**: confirm both `After=` and `Wants=network-online.target` are set, and `systemctl enable systemd-networkd-wait-online` (netplan+networkd environments) or `NetworkManager-wait-online`. If it is still flaky, add `ExecStartPre=/bin/sleep 5` as a fallback.
- **Runs fine manually but fails as a service**: nine times out of ten it is an environment problem—remember that systemd does not read `.bashrc`. Compare against the errors in `journalctl -u ros-robot` and check the paths the script sources (devel vs install).
- **Logs filling the disk**: the journal can occupy a considerable amount of disk by default. Edit `/etc/systemd/journald.conf` to set `SystemMaxUse=500M`, then `sudo systemctl restart systemd-journald`; for managing rosout file logs see [Section 04](04_performance_and_ops.md).
- **`status` shows an activating (auto-restart) loop**: the service is stuck in a crash-restart loop; use `journalctl -u ros-robot -b -p err` to find the cause of the first crash—a missing device is common (configure `Requires=dev-*.device` to make the cause explicit).
- **Changed the service file but nothing happened**: you forgot `daemon-reload`.
