# Romindly ROS1 Learning Kit · Documentation Index

> 中文版 / Chinese: [README.md](../README.md)

A hands-on ROS1 (Noetic) tutorial series for customer engineers. The standard runtime environment is an x86_64 edge computing unit + Ubuntu 20.04, and the standard workspace path is `~/ws_romindly`.

## Learning Path

| Chapter | Content | Status |
| --- | --- | --- |
| [00 Environment Setup](00_setup/01_install_ubuntu2004_ros_noetic.md) | Noetic installation, workspace and code checkout, common tools cheat sheet | ✅ |
| [10 ROS1 Basics](10_ros1_basics/README.md) | Core concepts overview, topic communication, launch and the parameter server | ✅ |
| [15 Simulation Basics](15_simulation/README.md) | TurtleBot3 trio + Gazebo: installing the models, keyboard teleop, observing sensor topics | ✅ |
| [20 2D SLAM](20_2d_slam/README.md) | gmapping / hector / cartographer / slam_toolbox comparison, mapping from bag recordings, saving maps | ✅ |
| [25 3D LiDAR SLAM](25_3d_lidar_slam/README.md) | A-LOAM principles, LIO-SAM, FAST-LIO hands-on and compute budgeting | ✅ |
| [30 Localization](30_localization/README.md) | AMCL principles and tuning, robot_localization EKF fusion, hdl_localization 3D localization | ✅ |
| [40 Navigation](40_navigation/README.md) | move_base architecture, costmap tuning, DWA vs TEB, navigation tuning checklist | ✅ |
| [45 Sensor Drivers](45_sensor_drivers/README.md) | rplidar / velodyne / realsense bring-up, fixed device names via udev rules | ✅ |
|[50 Deployment & Ops](50_deployment/README.md) | Field deployment, systemd autostart, Docker, performance | ✅ |

Status legend: ✅ complete and ready to follow along; 🚧 in progress — the chapter README lists the planned sections.

## Recommended Path for Beginners

Just follow the chapters in numerical order: first complete **00 Environment Setup** (install Noetic, pull all the code, and get `roslaunch urdf_demo display.launch` running), then go through **10 ROS1 Basics** to build up the concepts of topics/nodes/launch, and move on to **15 Simulation Basics** — all subsequent experiments for mapping (20), localization (30), and navigation (40) run on that simulation environment, and once mastered they transfer to the physical robot (45 Sensor Drivers). Finally, use **50 Deployment and Operations** to harden the system for delivery. Readers with prior ROS experience who care only about a specific topic can jump straight to the corresponding chapter; the prerequisites are stated at the beginning of each chapter.
