# 03 · hector_slam in Practice

> 中文版 / Chinese: [03_hector_slam实战.md](../../20_2D建图/03_hector_slam实战.md)

## Goals

- Understand the principle and applicability boundaries of hector_slam's odometry-free mapping
- Master how to launch `hector_mapping` and the key TF configuration points (the easiest pitfall when getting started with hector)
- Be able to handle localization loss caused by fast rotation

## Principles in Brief

hector_slam (developed at TU Darmstadt, package name `hector_mapping`) is a pure scan-matching solution: each laser scan is aligned directly to the current occupancy grid map via the **Gauss-Newton method**, aided by a **multi-resolution map pyramid** (align on the coarse map first, then refine on the fine map) to avoid local extrema. It uses no odometry at all, therefore:

- **Suitable scenarios**: quick handheld-lidar scanning, low-cost chassis without encoders, platforms with extremely unreliable odometry such as tracked vehicles, disaster-rescue robots (hector's origin).
- **The cost**: localization depends entirely on the overlap between two adjacent laser scans. The higher the lidar frame rate and the slower the motion, the more stable the matching; during **fast rotation**, inter-frame overlap drops sharply, localization is easily lost, and there is no odometry to fall back on. No loop-closure detection, so low-feature scenes such as long corridors will drift.

Source reading: `Romindly-Dev/hector_slam` (upstream tu-darmstadt-ros-pkg); parameter defaults are in `hector_mapping/src/HectorMappingRos.cpp`, and the reference launch is `hector_mapping/launch/mapping_default.launch`.

## Key TF Configuration Points (read this first)

The TF that hector outputs is determined by two parameters — think this through before wiring up a real robot:

- `pub_map_odom_transform` (default `true`): publishes the `map → odom` TF. This requires that `odom → base_link` **already exists** in the TF tree (published by anyone).
- If the chassis has **no odometry at all**: set `odom_frame` to the same value as `base_frame` (e.g., both `base_link`), and hector will publish `map → base_link` directly — the simplest possible TF tree.
- `base_frame` defaults to `base_link` and `odom_frame` to `odom`; in `mapping_default.launch` they are changed to `base_footprint` / `nav`. **When applying that launch file to your own robot you must change them back to your own frame names**, otherwise the TF will not line up and the map will not move.
- hector also publishes `map → scanmatcher_frame` (for debugging the matching result), which can be disabled via `pub_map_scanmatch_transform`.

## Step-by-Step

Run `export TURTLEBOT3_MODEL=burger` first in every terminal. The TB3 simulation has odometry built in; both modes are demonstrated here.

### 1. Start the simulation

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### 2a. TB3 quick launch

```bash
sudo apt install ros-noetic-hector-slam      # First time only
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector
```

Once RViz opens, the map appears immediately (hector does not wait for a movement threshold — it registers the very first scan). Expected log:

```text
HectorSM map lvl 0: cellLength: 0.05 res x: 2048 res y: 2048
HectorSM map lvl 1: cellLength: 0.1 res x: 1024 res y: 1024
[ INFO] [...]: HectorSlamRos node started.
```

### 2b. Generic launch (simulating the "no-odometry" real-robot setup)

```bash
rosrun hector_mapping hector_mapping scan:=scan \
    _base_frame:=base_footprint _odom_frame:=base_footprint \
    _pub_map_odom_transform:=false \
    _map_resolution:=0.05 _map_size:=1024 \
    _map_update_distance_thresh:=0.1 _map_update_angle_thresh:=0.06
```

Here hector publishes `map → base_footprint` directly, with no dependence on wheel odometry — this is exactly the handheld-lidar mapping setup.

**Complete handheld-lidar mapping template** (real robot, RPLIDAR as an example; copy as is):

```xml
<launch>
  <!-- 1. Lidar driver (replace with your actual lidar) -->
  <node pkg="rplidar_ros" type="rplidarNode" name="rplidar">
    <param name="frame_id" value="laser"/>
  </node>

  <!-- 2. Handheld, no chassis: provide a fixed TF between base_link and laser -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser"
        args="0 0 0 0 0 0 base_link laser 50"/>

  <!-- 3. hector: odometry-free mode, publishes map->base_link directly -->
  <node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">
    <param name="base_frame" value="base_link"/>
    <param name="odom_frame" value="base_link"/>
    <param name="pub_map_odom_transform" value="false"/>
    <param name="map_resolution" value="0.05"/>
    <param name="map_size" value="2048"/>
    <param name="map_update_distance_thresh" value="0.1"/>
    <param name="map_update_angle_thresh" value="0.06"/>
    <param name="laser_max_dist" value="12.0"/>
  </node>
</launch>
```

Self-check after startup:

```bash
rostopic hz /scan                 # Handheld mapping depends heavily on frame rate; below 10 Hz, move slower
rostopic echo /poseupdate -n1     # hector's current pose estimate (with covariance)
rosrun tf tf_echo map base_link   # Moving the lidar should show the pose changing
```

### 3. Teleoperate and save

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
# Move slowly while mapping, turn extra slowly; when done:
rosrun map_server map_saver -f ~/maps/tb3_world_hector
```

hector also offers a built-in save method (which saves the trajectory too): `rostopic pub syscommand std_msgs/String "savegeotiff"` (requires the hector_geotiff node to be running). For the teaching stage, map_saver is enough.

## Parameters in Detail

Default values verified against `HectorMappingRos.cpp` (values in parentheses are the overrides in `mapping_default.launch`):

| Parameter | Default | Purpose | Tuning advice |
| --- | --- | --- | --- |
| `map_resolution` | 0.025 (launch: 0.05) | Finest-level map resolution (m) | 0.05 suffices for navigation maps; 0.025 matches more accurately but quadruples CPU/memory |
| `map_size` | 1024 (launch: 2048) | Map side length (cells); actual size = size×resolution | 0.05×2048 ≈ 102 m square; hector **does not auto-expand the map** — allocate enough up front for large scenes |
| `map_start_x` / `map_start_y` | 0.5 / 0.5 | Relative position of the start point within the map (0–1) | 0.5 = start building from the map center; bias to e.g. 0.2 when scanning in one direction |
| `map_multi_res_levels` | 3 (launch: 2) | Number of levels in the multi-resolution pyramid | More levels tolerate fast motion better; 2–3 are both fine |
| `map_update_distance_thresh` | 0.4 (m) | Write the current scan into the map only after translating beyond this value | Reduce to 0.1–0.2 for finer map updates; note this only controls **map writing** — pose estimation runs on every frame |
| `map_update_angle_thresh` | 0.9 (launch: 0.06) (rad) | Rotation threshold for map writing | 0.06–0.1 recommended; the default 0.9 leaves the map un-updated for a long time during rotation |
| `update_factor_free` | 0.4 | Probability update factor for free cells (<0.5 reduces occupancy probability) | With many dynamic obstacles (pedestrians), lower to 0.3–0.35 so residual shadows fade faster |
| `update_factor_occupied` | 0.9 | Probability update factor for occupied cells | Usually left alone |
| `laser_min_dist` / `laser_max_dist` | 0.4 / 30.0 (m) | Laser distance range participating in matching | Set according to the lidar's actual reliable range; TB3 LDS: max 3.5 |
| `laser_z_min_value` / `laser_z_max_value` | -1.0 / 1.0 (m) | Point filtering by height relative to the lidar (for tilted mounting / roll) | Keep defaults for level mounting |
| `pub_map_odom_transform` | true | Whether to publish map→odom | See the TF points above |
| `base_frame` / `map_frame` / `odom_frame` | base_link / map / odom | Frame names | Fill in per your actual TF tree; without odometry, odom_frame = base_frame |
| `use_tf_scan_transformation` | true | Use TF to transform laser points into base_frame | Keep true, and ensure the `base→laser` TF is correct |
| `scan_topic` | scan | Laser topic name | — |
| `scan_subscriber_queue_size` | 5 | Laser subscription queue | Increase when replaying rosbags offline |
| `map_pub_period` | 2.0 (s) | `/map` publish period | Only affects visualization |
| `pub_odometry` | false | Publish the matched pose as nav_msgs/Odometry (topic `scanmatch_odom`) | Enable when feeding hector to other modules as a "laser odometry" source |

## FAQ

**Q1: After a fast rotation the whole map is skewed / the robot "teleports"?**
This is hector's number-one problem: during rotation, inter-frame overlap is insufficient, Gauss-Newton converges to a wrong solution, and there is no odometry to pull it back. Countermeasures, in priority order:

1. **Control angular speed**: turn slowly when handheld / teleoperating (the TB3 simulated lidar is only 5 Hz; keep angular speed ≤ 0.3 rad/s);
2. Use a high-frame-rate lidar (hector only shines at 20–40 Hz);
3. Increase `map_multi_res_levels` — coarser map levels tolerate large displacements better;
4. With an IMU, use `hector_imu_attitude_to_tf` to provide an attitude prior;
5. Once lost, the only option is to rebuild: `rostopic pub -1 /syscommand std_msgs/String "reset"` clears the map and starts over.

**Q2: After launch, RViz shows a map but the robot does not move / TF errors?**
Nine times out of ten it is mismatched frame names: confirm `base_frame` matches the URDF (TB3 uses `base_footprint`), and check with `rosrun tf view_frames` or `rosrun rqt_tf_tree rqt_tf_tree` that the TF tree is connected from `map` all the way to the lidar frame.

**Q3: Logs flooded with `lookupTransform ... extrapolation into the past/future`?**
The lidar driver and the host are not time-synchronized, or TF publishing is delayed. On real robots, do NTP/chrony time sync first; for rosbag playback add `--clock` and set `use_sim_time=true`.

**Q4: The map keeps stretching longer and longer in a long corridor?**
The corridor direction lacks geometric constraints, so pure scan matching cannot perceive forward motion (the "white wall problem"). hector has no fix; switch to a solution with odometry constraints (gmapping/slam_toolbox), or place feature objects such as cardboard boxes in the corridor.

**Q5: Can hector be used together with navigation?**
Yes: with `pub_map_odom_transform:=true` and a chassis that has odom, the TF tree is identical to other SLAMs and move_base stacks on directly. But in production, it is recommended to use hector only for mapping or as laser odometry, leaving localization to AMCL / slam_toolbox.
