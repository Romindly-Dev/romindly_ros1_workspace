# 02 · gmapping in Practice

> 中文版 / Chinese: [02_gmapping实战.md](../../20_2D建图/02_gmapping实战.md)

## Goals

- Complete a full mapping run with gmapping in the TurtleBot3 simulation and save the map
- Understand the physical meaning of gmapping's (RBPF particle filter) core parameters and how to tune them
- Be able to diagnose the typical causes of poor map quality

## Principles in Brief

gmapping is the classic implementation of RBPF (Rao-Blackwellized Particle Filter) SLAM; the ROS package is `slam_gmapping` (wrapping the `openslam_gmapping` algorithm library). Each particle carries a complete trajectory hypothesis and its own map; whenever the robot moves beyond the `linearUpdate`/`angularUpdate` thresholds, one round of "odometry propagation → laser scan-matching correction → scoring → resampling as needed" is executed. It has **no loop-closure detection** and relies on odometry and scan matching to suppress drift, so it is sensitive to odometry quality.

Source reading: `Romindly-Dev/slam_gmapping` (upstream ros-perception); parameter defaults are in `SlamGMapping::init()` in `gmapping/src/slam_gmapping.cpp`.

## Step-by-Step

Open a new terminal for each step below, and run `export TURTLEBOT3_MODEL=burger` first in each.

### 1. Start the simulation

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Gazebo opens the hexagonal world; `odom received!` in the terminal means the chassis is ready.

### 2. Start gmapping (with RViz)

TB3 shortcut:

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Expected output (excerpt):

```text
[ INFO] [...]: Laser is mounted upwards.
Registering First Scan
```

RViz opens automatically and shows the map gradually unfolding around the robot.

Generic method (for real robots / non-TB3 chassis, provided `/scan` and the `odom→base_link` TF are ready):

```bash
rosrun gmapping slam_gmapping scan:=scan _base_frame:=base_footprint _odom_frame:=odom \
    _map_update_interval:=2.0 _maxUrange:=3.0 _particles:=30
```

For real-robot projects, it is recommended to solidify this into a launch file (parameters are visible at a glance and easy to version-control):

```xml
<launch>
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <remap from="scan" to="/scan"/>
    <param name="base_frame" value="base_link"/>   <!-- Change to match your chassis -->
    <param name="odom_frame" value="odom"/>
    <param name="map_frame"  value="map"/>
    <param name="particles" value="50"/>
    <param name="linearUpdate" value="0.3"/>
    <param name="angularUpdate" value="0.25"/>
    <param name="map_update_interval" value="2.0"/>
    <param name="maxUrange" value="10.0"/>         <!-- 80% of the lidar's reliable range -->
    <param name="minimumScore" value="50"/>
    <param name="delta" value="0.05"/>
    <param name="srr" value="0.1"/> <param name="srt" value="0.2"/>
    <param name="str" value="0.1"/> <param name="stt" value="0.2"/>
  </node>
</launch>
```

Manual RViz configuration: set Fixed Frame to `map`, and add `Map` (topic `/map`) and `LaserScan` (topic `/scan`) displays.

Self-check after startup (new terminal):

```bash
rosnode info /slam_gmapping        # Confirm it subscribes to /scan and /tf, and publishes /map
rosrun tf tf_echo map odom          # gmapping has started publishing the correction
rostopic hz /map                    # Roughly one frame per map_update_interval seconds
```

### 3. Teleoperate to map

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Driving tips: linear speed ≤ 0.15 m/s, **turn slowly** (angular speed ≤ 0.5 rad/s); drive a full loop along the walls, and finally return near the starting point to check for misalignment at the closure.

### 4. Save the map

Once you are satisfied with the map, **do not close any node**; in a new terminal run:

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/tb3_world_gmapping
```

Expected output:

```text
[ INFO] [...]: Waiting for the map
[ INFO] [...]: Received a 384 X 384 map @ 0.050 m/pix
[ INFO] [...]: Writing map occupancy data to /home/iot/maps/tb3_world_gmapping.pgm
[ INFO] [...]: Writing map occupancy data to /home/iot/maps/tb3_world_gmapping.yaml
[ INFO] [...]: Done
```

`map_saver` merely subscribes to `/map` once and writes it to disk; after saving, you can Ctrl+C to shut everything down. Verify:

```bash
rosrun map_server map_server ~/maps/tb3_world_gmapping.yaml   # Re-publish /map and view it in RViz
```

For the meaning of the `map.yaml` fields, see [Doc 01](./01_principles_and_selection.md#occupancy-grid-maps). Note that the `image` field is a relative path: when moving map files, move the pgm and yaml together.

## Parameters in Detail

Default values verified against `slam_gmapping.cpp` (Noetic branch). Listed in order of importance:

| Parameter | Default | Purpose | Tuning advice |
| --- | --- | --- | --- |
| `particles` | 30 | Particle count; one map per particle | 30 suffices for small scenes; raise to 50–80 for large scenes / poor odometry; CPU and memory grow linearly |
| `linearUpdate` | 1.0 (m) | Process a laser scan only after translating beyond this value | Often changed to 0.2–0.5 for denser updates and a more detailed map, at the cost of more computation |
| `angularUpdate` | 0.5 (rad) | Process a laser scan only after rotating beyond this value | Often changed to 0.2–0.25 to reduce map misalignment during turns |
| `temporalUpdate` | -1.0 (s) | Force an update after being stationary this long; negative disables | Usually keep disabled; set 3–5 s in dynamic environments |
| `map_update_interval` | 5.0 (s) | Publish period of the `/map` topic | Only affects visualization refresh, not accuracy; 2.0 looks better |
| `maxUrange` | equals maxRange | Maximum laser distance used for **mapping** | Set to 80% of the lidar's reliable range (3.0–3.5 for the TB3 LDS); too large introduces far-range noise |
| `maxRange` | scan.range_max − 0.01 | Maximum distance used for **clearing free space** | Usually the lidar's nominal range; keep maxUrange < maxRange |
| `srr` | 0.1 | Odometry noise model: translation error caused by translation | Increase to about 0.2 if odometry slips badly (carpet / differential wheels) |
| `srt` | 0.2 | Translation error caused by rotation | Same as above; larger = trust odometry less, rely more on scan matching |
| `str` | 0.1 | Rotation error caused by translation | Usually left alone |
| `stt` | 0.2 | Rotation error caused by rotation | Increase somewhat if the map warps after turns |
| `minimumScore` | 0 | Fall back to odometry when the scan-matching score is below this | Set 50–200 in large open areas (laser cannot reach walls) to keep matching from jumping around |
| `delta` | 0.05 (m) | Map resolution | 0.05 in general; 0.025 for fine scenes (about 4x the cost) |
| `xmin/ymin/xmax/ymax` | ±100.0 (m) | Initial map extent | Shrink to save memory if the scene size is known (gmapping auto-expands the map) |
| `iterations` | 5 | Scan-matching optimization iterations per frame | Usually left alone |
| `sigma` / `lsigma` | 0.05 / 0.075 | Gaussian variance for endpoint scoring / likelihood computation | Usually left alone |
| `kernelSize` | 1 | Correspondence search window (cells) during matching | Can be 2–3 when odometry is very poor, at high cost |
| `lskip` | 0 | Number of laser beams skipped per frame (0 = use all) | Set 1–2 when CPU is tight |
| `resampleThreshold` | 0.5 | Trigger resampling when Neff/N drops below this ratio | Usually left alone |
| `throttle_scans` | 1 | Process 1 of every n laser scans | Set 2 for high-frame-rate lidars (>15 Hz) |
| `transform_publish_period` | 0.05 (s) | Publish period of the `map→odom` TF | Usually left alone |
| `occ_thresh` | 0.25 | Probability threshold for judging a cell occupied | Usually left alone |

TB3's `turtlebot3_slam` package already overrides most key parameters in `config/gmapping_params.yaml` (e.g., `maxUrange: 3.0`, `linearUpdate: 1.0`, `map_update_interval: 2.0`); when tuning on a real robot, your own explicit overrides in launch/yaml take precedence.

## FAQ

**Q1: The map shows ghosting / walls become double-layered?**
The typical cause is **odometry drift**: wheel slippage, inaccurate wheel-diameter parameters, or a lagging timestamp on the `odom→base_link` TF. Countermeasures: calibrate wheel diameter and wheelbase; increase `srr/srt/stt` to trust odometry less; drive slower.

**Q2: A whole chunk of the map rotates out of alignment after a turn?**
**Turning too fast** is gmapping's most common failure — too much rotation between two updates makes scan matching fall into a local extremum. Countermeasures: keep angular speed below 0.5 rad/s while teleoperating; reduce `angularUpdate` to 0.2; increase the particle count somewhat.

**Q3: Error `Scan Matching Failed, using odometry.`?**
Occasional occurrences are normal (that frame falls back to odometry). If it persists, the environment has too few features (long corridor, large empty hall) or `maxUrange` is too small for the laser to reach the walls — gmapping is inherently weak in such scenes; consider switching to slam_toolbox / cartographer.

**Q4: `map_saver` stays at `Waiting for the map`?**
`/map` has not been published yet (just started, `map_update_interval` not yet reached) or the topic name is wrong. Confirm with `rostopic list | grep map`; for topics under a namespace use `map_saver map:=/xxx/map`.

**Q5: Back at the start, the closure is off by tens of centimeters?**
gmapping has no loop-closure detection, so accumulated error over a large loop cannot be eliminated. Small misalignments are acceptable (navigation is backstopped by AMCL); for large ones, re-scan at lower speed or switch to [slam_toolbox](./04_slam_toolbox.md), which has loop closure.
