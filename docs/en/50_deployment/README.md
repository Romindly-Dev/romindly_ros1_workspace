# 50 Deployment and Operations

> 中文版 / Chinese: [README.md](../../50_部署运维/README.md)

This chapter addresses the "after development is done" problem: deploying the system that runs on your development machine onto the Romindly Mind edge computing unit, achieving **power-on-and-go operation, reproducible environments, and long-term unattended maintainability**. This is the final, delivery-oriented chapter.

## Section List

| Section | Content | Companion Code |
| --- | --- | --- |
| [01 Real-Machine Deployment Checklist](01_deployment_checklist.md) | Complete checklist for deploying an edge unit from scratch: system settings, ROS installation (including the expired apt key fix), chrony time synchronization, two workspace deployment routes, install space, multi-machine network configuration, acceptance tests | — |
| [02 systemd Autostart on Boot](02_systemd_autostart.md) | Line-by-line walkthrough of `ros_env.sh` and `ros-robot.service`, installation and enablement, journalctl troubleshooting, dual-service architecture, graceful shutdown | [romindly_robot_bringup](https://github.com/Romindly-Dev/romindly_robot_bringup) |
| [03 Docker Deployment](03_docker_deployment.md) | Freezing dependencies in an image after Noetic EOL: section-by-section Dockerfile walkthrough, device/network/display passthrough, offline distribution, container autostart, compose orchestration | [docker/](https://github.com/Romindly-Dev/romindly_ros1_workspace/blob/main/docker/README.md) |
| [04 Performance Optimization and Operations](04_performance_and_ops.md) | Resource monitoring and bottleneck identification, per-module load-reduction summary, CPU pinning, log management, remote operations and black-box recording, upgrade strategy under EOL | — |

## Development Machine vs. Edge Unit: Deployment Differences

Same codebase, but the two environments have completely different operating assumptions. Align on these differences before deploying; each section of this chapter addresses the corresponding problem:

| Dimension | Development Machine | Edge Unit (Romindly Mind) | Section |
| --- | --- | --- | --- |
| GUI | Has a desktop; rviz/rqt opened at will | Usually no monitor (headless); rviz runs remotely or is not installed | 01 / 04 |
| Startup | Manually open a terminal and `roslaunch` | Starts automatically on power-up, unattended; must auto-restart on crash | 02 / 03 |
| Power | Stable; casual sleep/suspend does not matter | Field power supply; **automatic sleep/suspend must be disabled** | 01 |
| Network | DHCP, IP changes at any time | Static IP; multi-machine ROS communication depends on fixed addresses and hostname resolution | 01 |
| Software environment | Frequently installed and modified; reinstall if broken | Frozen after deployment; dependencies must be reproducible (all the more so under EOL) | 01 / 03 |
| System updates | Automatic updates do not matter | Automatic updates may pull packages and reboot in the middle of the night; **must be disabled** | 01 |
| Time | NTP synchronized automatically | May have no internet access; multi-machine/multi-sensor timestamps require chrony synchronization on the local network | 01 |
| Logs | Just look at the terminal | Only journalctl / rosout files, which accumulate and eventually fill the disk | 02 / 04 |

## Prerequisites

- Chapters 00–45 completed: system functionality has been verified in the development environment (simulation or bench setup).
- [Chapter 45 udev rules](../45_sensor_drivers/04_udev_and_device_management.md) are configured, with device names fixed as `/dev/rplidar`, `/dev/base_serial`, etc.—autostart on boot strongly depends on fixed device names.
- An edge unit ready for deployment (Ubuntu 20.04 x86_64), accessible via a monitor or ssh.

## Suggested Reading Order

Work through 01 → 02 → 03 in order, and the machine reaches the "power-on-and-go" delivery state; consult 04 as needed once the system is running. Readers delivering with Docker only may skip the bare-metal ROS installation part of 01, but the network, time synchronization, and acceptance checklist still apply.
