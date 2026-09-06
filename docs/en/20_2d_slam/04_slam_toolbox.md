# 04 · slam_toolbox in Practice

> 中文版 / Chinese: [04_slam_toolbox实战.md](../../20_2D建图/04_slam_toolbox实战.md)

## Goals

- Understand why slam_toolbox is positioned as the "first choice for production" and its five working modes
- Distinguish the sync / async nodes and complete a mapping run in simulation
- Master the difference between the `.posegraph` serialized map and the `map_saver` static map, and their respective uses
- Get to know pure localization mode, as groundwork for [30_Localization](../30_localization/README.md)

## Principles in Brief

slam_toolbox (developed by Steve Macenski, the default SLAM for ROS2/Nav2) is a rewrite based on Karto's scan-to-map front end: pose graph + Ceres nonlinear-optimization back end, with loop-closure detection. Compared with the two previous solutions, its engineering capability is a qualitative leap:

- **Online mapping** (online sync/async): regular real-time mapping;
- **Offline mapping** (offline): replay a rosbag and process every frame without dropping any;
- **Continued mapping (map extension)**: load a `.posegraph` and continue mapping on top of the old map — if the customer site adds a new warehouse area, you don't have to re-scan from scratch;
- **Pure localization** (localization mode): "elastic localization" on an existing pose graph; new scans only add temporary constraints without changing the map — can replace AMCL;
- **lifelong** (experimental): add and remove nodes during long-term operation to adapt to environmental changes.

One package covering the full "mapping → continuation → localization" lifecycle, plus the large-scene accuracy that loop closure brings — that is why it is the first choice for production.

**sync vs async nodes**:

- `sync_slam_toolbox_node` (online_sync): queues and processes every qualifying scan, **never dropping frames**; map quality first — processing may lag when the robot moves fast. Suited to offline processing and quality-critical mapping;
- `async_slam_toolbox_node` (online_async): always processes the **latest frame**, dropping older frames when it cannot keep up, guaranteeing real-time behavior. Suited to mapping on the fly on onboard computers with limited compute.

Both nodes have identical parameters; only the scheduling strategy differs. Use sync for teaching and simulation; real deployments generally choose async.

Source reading: `Romindly-Dev/slam_toolbox` (upstream SteveMacenski, noetic branch). Config files are in `slam_toolbox/config/mapper_params_*.yaml`, launch files in `slam_toolbox/launch/`.

## Step-by-Step

```bash
sudo apt install ros-noetic-slam-toolbox     # First time only
```

Run `export TURTLEBOT3_MODEL=burger` first in every terminal.

### 1. Start the simulation and slam_toolbox

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

New terminal (the default config has `base_frame: base_footprint` and `scan_topic: /scan`, which matches TB3 exactly — no changes needed):

```bash
roslaunch slam_toolbox online_sync.launch
```

Expected output (excerpt):

```text
[ INFO] [...]: Solver plugin loaded: solver_plugins::CeresSolver
Registering sensor: [Custom Described Lidar]
```

Generic setup for real robots / custom configs (recommended: copy the official yaml into your own package and load it, rather than passing parameters on the command line):

```xml
<launch>
  <node pkg="slam_toolbox" type="sync_slam_toolbox_node" name="slam_toolbox" output="screen">
    <rosparam command="load" file="$(find my_robot_bringup)/config/slam_toolbox.yaml"/>
    <param name="base_frame" value="base_link"/>   <!-- Overrides the yaml; change to match your chassis -->
    <param name="max_laser_range" value="12.0"/>
  </node>
</launch>
```

Self-check after startup:

```bash
rosnode info /slam_toolbox | grep -A4 Services   # Should show services such as save_map / serialize_map
rosrun tf tf_echo map odom                       # Confirm map->odom is being published
```

### 2. RViz and teleoperation

```bash
rosrun rviz rviz    # Fixed Frame=map, add Map(/map) and LaserScan(/scan)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Note the defaults `minimum_travel_distance: 0.5` / `minimum_travel_heading: 0.5` — a new node is only added after the robot moves 0.5 m or rotates 0.5 rad, so the map updating looking "sluggish" at the start is normal. Drive a full loop back to the start and watch the whole map get "pulled straight" the moment the loop closes (loop-closure messages appear in the log).

RViz can load a dedicated panel: Panels → Add New Panel → `SlamToolboxPlugin`, which provides visual controls for save/serialize/pause and more.

### 3. Save the map (two methods — do both)

```bash
# Method 1: static grid map (for map_server/AMCL/move_base)
rosrun map_server map_saver -f ~/maps/tb3_world_toolbox
# Equivalent service: rosservice call /slam_toolbox/save_map "name: {data: '/home/iot/maps/tb3_world_toolbox'}"

# Method 2: serialized pose graph (for slam_toolbox's own continued mapping / pure localization)
rosservice call /slam_toolbox/serialize_map "filename: '/home/iot/maps/tb3_world_toolbox'"
```

Method 2 produces `tb3_world_toolbox.posegraph` + `tb3_world_toolbox.data`. **The difference**: the PGM exported by `map_saver` is a "dead" raster snapshot, only good for viewing and localization; the `.posegraph` retains all nodes, scans, and constraints — a "live" project file that can be reopened to continue mapping or do elastic localization. For product delivery, archive both.

### 4. Continued mapping (map extension)

Edit the config (or use the RViz panel's Deserialize): uncomment in `mapper_params_online_sync.yaml`:

```yaml
map_file_name: /home/iot/maps/tb3_world_toolbox   # Without extension
map_start_at_dock: true          # Continue from the mapping start point; or use map_start_pose: [x, y, theta]
```

Restart `online_sync.launch`; once the old map loads you can continue scanning new areas. You can also call the `/slam_toolbox/deserialize_map` service to load dynamically.

### 5. Pure localization mode (preview)

```bash
roslaunch slam_toolbox localization.launch
```

Before use, edit `config/mapper_params_localization.yaml`: change `mode: mapping` to `mode: localization` (note: in the source repo this line still defaults to `mapping`, with `localization` only in the comment — **you must change it manually**), and set `map_file_name` to point at the `.posegraph`. In this mode the map is no longer modified; new scans are used only for elastic-matching localization, and re-localization via `/initialpose` (RViz 2D Pose Estimate) is supported. The comparison with AMCL and selection guidance is covered in Chapter 30.

## Parameters in Detail

Verified against `config/mapper_params_online_sync.yaml` (async is identical); only key items listed:

| Parameter | Default | Purpose | Tuning advice |
| --- | --- | --- | --- |
| `mode` | mapping | mapping / localization | Change to localization for localization deployments |
| `solver_plugin` | CeresSolver | Back-end optimizer | Keep the default |
| `resolution` | 0.05 | Map resolution (m) | Same as other solutions; 0.05 in general |
| `max_laser_range` | 20.0 | Maximum laser distance used for mapping | Set per your lidar; 3.5 for TB3; too large introduces noise |
| `map_update_interval` | 5.0 (s) | /map publish period | Only affects visualization |
| `minimum_travel_distance` | 0.5 (m) | Translation threshold for adding a graph node | Lower to 0.3 in small rooms; smaller = bigger graph, higher CPU |
| `minimum_travel_heading` | 0.5 (rad) | Rotation threshold for adding a graph node | Can lower to 0.25 to improve detail in turns |
| `minimum_time_interval` | 0.5 (s) | Minimum time between two processed frames | Usually left alone |
| `scan_buffer_size` | 10 | Sliding-window frame count for scan-to-map matching | Increase when odometry is poor |
| `do_loop_closing` | true | Loop-closure switch | Can be temporarily disabled while tuning the front end (same idea as cartographer) |
| `loop_search_maximum_distance` | 3.0 (m) | Loop-closure candidate search radius | Increase for large scenes / heavy drift (together with the next item) |
| `loop_match_minimum_chain_size` | 10 | Minimum node-chain length to constitute a loop | Increase if false loop closures are frequent; decrease if loops rarely trigger |
| `loop_match_minimum_response_coarse/fine` | 0.35 / 0.45 | Loop-matching response thresholds | Raise if false loop closures (map gets smeared); lower if loops are missed |
| `correlation_search_space_dimension` | 0.5 (m) | Search window for ordinary matching | Increase to 0.8–1.0 when odometry is poor |
| `transform_publish_period` | 0.02 (s) | map→odom TF period | Usually left alone |
| `enable_interactive_mode` | true | Interactive map editing in RViz (drag nodes) | Turn off in production to save memory |
| `stack_size_to_use` | 40000000 | Stack space needed to serialize large maps | Increase if serializing a very large map crashes |
| `debug_logging` | false | Verbose logging | Enable when troubleshooting |

## FAQ

**Q1: TF errors right at startup / no map appears?**
Confirm `base_frame` matches the actual robot (default `base_footprint`; many chassis use `base_link`) and that the `odom→base` TF exists. Like gmapping, slam_toolbox **requires odometry**.

**Q2: The map "jumps" the moment a loop closes — is that normal?**
Normal, and exactly where the value lies — the global optimization redistributes the accumulated error across the historical trajectory in one go. If things get worse after the jump (a false loop closure), raise the `loop_match_minimum_response_*` thresholds.

**Q3: Can maps saved by serialize and map_saver be converted into each other?**
The `.posegraph` can be re-rasterized into a PGM at any time (load it, then run map_saver); the reverse is impossible — the PGM has lost the pose-graph information and cannot be extended. So **serialize immediately after finishing a map**.

**Q4: deserialize complains the file is not found?**
The path in `map_file_name` and in the service call must be an **absolute path without the `.posegraph` extension**; both the `.posegraph` and `.data` files must exist in the same directory.

**Q5: CPU usage climbs the longer you map?**
An inherent property of graph-optimization solutions (more nodes = more expensive optimization). Countermeasures: increase `minimum_travel_distance/heading` somewhat to control node density; use the async node on weak onboard compute; for very large scenes, map in sections and merge with `merge_maps_kinematic`.
