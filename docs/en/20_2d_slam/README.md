# 20 · 2D Lidar SLAM

> 中文版 / Chinese: [README.md](../../20_2D建图/README.md)

This chapter covers SLAM mapping with a 2D lidar. After completing it, you should be able to build and save maps in both simulation and on a real robot using four mainstream solutions, and be able to select a solution and perform basic parameter tuning according to project requirements.

## Prerequisites

- Completed [10_ROS1 Basics](../10_ros1_basics/README.md) and [15_Simulation Primer](../15_simulation/README.md)
- TurtleBot3 simulation and SLAM-related packages installed:

```bash
sudo apt install ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-slam \
                 ros-noetic-turtlebot3-teleop ros-noetic-map-server
```

- All demos in this chapter use TurtleBot3 Burger + the `turtlebot3_world` simulation environment.

## Chapter Contents

| No. | Document | Contents |
| --- | --- | --- |
| 01 | [Mapping Principles and Solution Selection](./01_principles_and_selection.md) | Occupancy grid maps, the SLAM problem definition, three technical approaches, input requirements, selection advice |
| 02 | [gmapping in Practice](./02_gmapping.md) | Complete particle-filter mapping workflow, detailed parameters, map saving and map.yaml |
| 03 | [hector_slam in Practice](./03_hector_slam.md) | Mapping without odometry, key TF configuration points, countermeasures for losing localization during fast rotation |
| 04 | [slam_toolbox in Practice](./04_slam_toolbox.md) | Preferred solution for production, sync/async, pose-graph serialization, pure localization mode |
| 05 | [cartographer in Practice](./05_cartographer.md) | Lua configuration system, offline mapping, loop-closure tuning approach |

## Four Solutions Compared on One Page

| Solution | Principle category | Requires odometry? | CPU usage | Large-scene performance | Recommended scenarios |
| --- | --- | --- | --- | --- | --- |
| gmapping | Particle filter (RBPF) | Yes (TF odom→base_link) | Medium (grows linearly with particle count) | Poor: no loop closure; long corridors / large loops easily misalign; map tears when particle count is insufficient | Small indoor scenes with decent odometry; teaching and getting started |
| hector_slam | High-frequency scan matching (multi-resolution Gauss-Newton) | No | Low | Poor: no loop closure, no odometry constraints; drifts easily in long corridors with few geometric features | Quick handheld-lidar scanning, chassis without encoders, high-frame-rate lidars (≥20 Hz) |
| slam_toolbox | Graph optimization (scan-to-map front end + Ceres back end, with loop closure) | Yes | Medium | Good: loop closure + global optimization; supports serialization and continued mapping of very large maps | First choice for production deployment; large warehouses/factories; continuous mapping or pure localization |
| cartographer | Graph optimization (submaps + branch-and-bound loop closure + global BA) | Optional (recommended) | Medium-high (background optimization threads) | Very good: submap mechanism + strong loop closure; best for very large scenes and multi-sensor fusion | Very large scenes, multi-sensor (IMU/odometry/multiple lidars) fusion, offline high-precision mapping |

> Note: karto (`slam_karto`) shares its origins with slam_toolbox (slam_toolbox is a rewrite of the Karto front end with substantial enhancements). On Noetic, just learn slam_toolbox directly; this chapter does not cover karto separately.

## Common Conventions

- Environment variable: run `export TURTLEBOT3_MODEL=burger` in every terminal first (recommended to add it to `~/.bashrc`).
- Save all maps to the `~/maps/` directory; the later chapters [30_Localization](../30_localization/README.md) and [40_Navigation](../40_navigation/README.md) will reuse them.
- Each document provides two launch methods: the TB3-wrapped `turtlebot3_slam.launch` (quick for teaching) and the generic way of launching the algorithm node directly (for porting to real robots).
