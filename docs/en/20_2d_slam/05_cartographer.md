# 05 · cartographer in Practice

> 中文版 / Chinese: [05_cartographer实战.md](../../20_2D建图/05_cartographer实战.md)

## Goals

- Install cartographer_ros via apt and understand its lua configuration system
- Complete online mapping in the TB3 simulation and master saving and converting pbstream files
- Use the offline node to map from a rosbag
- Establish the tuning methodology of "tune the front end with loop closure off first, then turn loop closure back on for the back end"

## Installation

**Install via apt, uniformly**:

```bash
sudo apt install ros-noetic-cartographer-ros ros-noetic-cartographer-rviz
```

Do not put the cartographer source into your main workspace: it does not use catkin_make but requires `catkin_make_isolated`, and it depends on abseil-cpp which must be compiled separately — it would drag down the build of the whole workspace. If you need to read the source, just clone our fork into a separate directory (read only, do not build):

```bash
git clone https://github.com/Romindly-Dev/cartographer_ros ~/reading/cartographer_ros
```

## Principles in Brief

cartographer (by Google) is the most thoroughly engineered implementation of the graph-optimization approach, in two layers:

- **Front end (Local SLAM, `TRAJECTORY_BUILDER_2D`)**: consecutive scans go through voxel filtering and Ceres scan matching, then get inserted into the current **submap**; once a submap has accumulated `num_range_data` frames it is finalized and the next one starts. Error within a submap is very small, but drift accumulates between submaps.
- **Back end (Global SLAM, `POSE_GRAPH`)**: loop-closure constraints between all scans and all finalized submaps are searched efficiently with a **branch-and-bound** algorithm, and global optimization (SPA) runs periodically to re-align the submaps.

This "loop closure at submap granularity" keeps it robust even in very large scenes, and is also why its CPU usage is on the high side (constraint search runs continuously in the background).

## The Lua Configuration System

cartographer does not use rosparam; all configuration lives in lua files, passed to the node via `-configuration_directory` + `-configuration_basename`. The structure of a typical config:

```lua
include "map_builder.lua"          -- Pull in defaults (installed at /opt/ros/noetic/share/cartographer/configuration_files/)
include "trajectory_builder.lua"

options = {                        -- ROS interface-layer options (consumed by cartographer_ros)
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",     -- Reference frame for pose estimation: must be the IMU frame if an IMU is used
  published_frame = "odom",        -- cartographer publishes map→published_frame
  odom_frame = "odom",
  provide_odom_frame = false,      -- Set false if the chassis already provides the odom TF
  use_odometry = true,             -- Subscribe to /odom as a front-end prior
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true      -- Algorithm layer: override defaults

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
```

The layering: the `options` table is the ROS bridge layer (what to subscribe to, which TF to publish); `MAP_BUILDER` / `TRAJECTORY_BUILDER_2D` / `POSE_GRAPH` are the algorithm layer — `include` pulls in the defaults, then you override items one by one as needed.

## Step-by-Step

Run `export TURTLEBOT3_MODEL=burger` first in every terminal.

### 1. Start the simulation and cartographer

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

TB3 shortcut (if your installed `turtlebot3_slam` version still keeps the cartographer config):

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer
```

If it complains it cannot find the launch/lua (newer turtlebot3_slam removed cartographer support), use the generic method: save the lua from the previous section as `~/carto_ws/config/tb3_2d.lua`, then build your own launch file:

```xml
<launch>
  <node pkg="cartographer_ros" type="cartographer_node" name="cartographer_node"
        args="-configuration_directory $(env HOME)/carto_ws/config
              -configuration_basename tb3_2d.lua" output="screen">
    <remap from="scan" to="/scan"/>
    <remap from="odom" to="/odom"/>
    <remap from="imu"  to="/imu"/>
  </node>
  <node pkg="cartographer_ros" type="cartographer_occupancy_grid_node"
        name="cartographer_occupancy_grid_node" args="-resolution 0.05"/>
</launch>
```

Note: the `/map` topic is rasterized and published by the separate `cartographer_occupancy_grid_node`; forget to start it and RViz shows no map (though TF and submaps work normally).

### 2. Teleoperate, observe, and save

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun rviz rviz   # Fixed Frame=map; you can also observe submaps with cartographer_rviz's Submaps display
```

Drive a full loop and notice the submaps getting subtly adjusted when a loop closure triggers. Saving goes along two tracks:

```bash
# A. Generic static map (for navigation):
rosrun map_server map_saver -f ~/maps/tb3_world_carto

# B. cartographer's native state (pbstream, containing the full pose graph, for pure localization / continued runs):
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '/home/iot/maps/tb3_world_carto.pbstream', include_unfinished_submaps: true}"

# A pbstream can also be converted to PGM+yaml:
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename ~/maps/tb3_world_carto.pbstream -map_filestem ~/maps/tb3_world_carto
```

### 3. Offline mapping (rosbag + offline node)

Record bags on site and build the map carefully back at the office — this is cartographer's recommended workflow:

```bash
# Record a bag on site (laser + odometry + IMU + TF):
rosbag record -O tb3_run1.bag /scan /odom /imu /tf /tf_static

# Offline mapping (runs through the whole bag at maximum speed, automatically producing <bagname>.pbstream):
rosrun cartographer_ros cartographer_offline_node \
    -configuration_directory ~/carto_ws/config \
    -configuration_basename tb3_2d.lua \
    -bag_filenames /home/iot/tb3_run1.bag
```

The offline node does not depend on real time, so you can rerun the same bag repeatedly with different parameters for comparison — exactly the foundation of the tuning approach in the next section. When recording, do **not** run another SLAM at the same time, to keep `map→odom` TF out of the bag (or filter it with `tf` during playback).

## Parameters in Detail (key items)

Defaults are the factory values in `configuration_files/*.lua` under the cartographer install directory:

| Parameter | Default | Purpose | Tuning advice |
| --- | --- | --- | --- |
| `options.use_odometry` | false | Subscribe to /odom as a front-end motion prior | With wheel odometry, definitely set true — front-end stability improves greatly |
| `options.tracking_frame` | — | Reference frame for pose estimation | Must be the IMU frame when using an IMU (`imu_link` on TB3); otherwise the base frame |
| `options.provide_odom_frame` | — | Have cartographer publish the odom frame itself | Set false if the chassis already provides the odom TF, otherwise TF conflicts |
| `TRAJECTORY_BUILDER_2D.use_imu_data` | true | Front end uses IMU attitude | Without an IMU you must explicitly set false, otherwise it waits for the IMU forever and produces no map |
| `TRAJECTORY_BUILDER_2D.num_accumulated_range_data` | 1 | How many laser messages to accumulate into one frame before matching | Keep 1 for single-line lidars; for multi-echo / segmented-publishing lidars, set per the driver |
| `TRAJECTORY_BUILDER_2D.min_range / max_range` | 0. / 30. | Valid laser distance range | Set per your lidar; 0.12 / 3.5 for TB3 |
| `TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching` | false | Run a correlative brute-force search before matching | Enable with poor odometry / no IMU — resists slippage, adds CPU |
| `TRAJECTORY_BUILDER_2D.submaps.num_range_data` | 90 | Frames per submap | A submap should be small enough to be unaffected by drift, yet large enough to be recognizable on its own; slow vehicles can go down to 35–60 |
| `TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians` | ~0.017 (1°) | Motion filter: frames with less motion are dropped | Reduce if the map blurs while stationary |
| `POSE_GRAPH.optimize_every_n_nodes` | 90 | Run global optimization every n inserted nodes; **0 = disable loop closure / back end** | Step one of tuning: set 0; when restoring, recommended ≈ 1–2x num_range_data |
| `POSE_GRAPH.constraint_builder.min_score` | 0.55 | Minimum matching score for a loop constraint | Raise to 0.65+ if false loop closures smear the map |
| `POSE_GRAPH.constraint_builder.sampling_ratio` | 0.3 | Sampling ratio of nodes participating in loop search | Lowering saves CPU but yields fewer loop closures |

## Tuning Approach: Front End First with Loop Closure Off, Then Turn It Back On

Loop-closure optimization masks front-end problems; tuning everything at once gives you nothing to hold on to. The standard procedure:

1. **Disable the back end**: `POSE_GRAPH.optimize_every_n_nodes = 0`, and run the same bag with the offline node;
2. **Tune the front end**: the goal is "essentially no ghosting even without loop closure" — check TF/timestamps, `use_odometry`, and distance ranges in order, then tune `use_online_correlative_scan_matching`, the ceres weights, and `submaps.num_range_data`;
3. **Enable the back end**: restore `optimize_every_n_nodes = 90` and tune only the loop-closure items (`min_score`, `sampling_ratio`, optimization frequency), watching whether loops trigger correctly and whether any false loop closures occur;
4. Change only one parameter at a time, and compare results on the same bag.

## FAQ

**Q1: The node starts, produces no map at all, and reports no error?**
The most common cause is `use_imu_data = true` with no IMU data — cartographer waits silently. Without an IMU set it to false; with one, check the topic remapping and `tracking_frame`.

**Q2: `Check failed: ... frame` or TF timeouts?**
`tracking_frame` / `published_frame` do not match the actual TF tree, or the URDF is not loaded. cartographer's validation of TF and timestamps is far stricter than gmapping's; on real robots, be sure to synchronize clocks.

**Q3: The map is good overall but has small local jaggies?**
Normal per-frame matching noise from the front end. You can lower your resolution expectations, or reduce the `motion_filter` thresholds and raise the ceres matching weights; at the navigation level it usually needs no treatment.

**Q4: CPU usage too high?**
Try in order: lower `POSE_GRAPH.constraint_builder.sampling_ratio` (0.3→0.1), increase `optimize_every_n_nodes`, disable `use_online_correlative_scan_matching`, raise the motion_filter thresholds.

**Q5: Want pure localization inside cartographer?**
Add `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }` in the lua and launch with `-load_state_filename xxx.pbstream`. That said, for ROS1 production localization we recommend AMCL or slam_toolbox localization instead — see [30_Localization](../30_localization/README.md).
