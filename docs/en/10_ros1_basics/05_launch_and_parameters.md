# 05 launch and the Parameter Server

> 中文版 / Chinese: [05_launch与参数服务器.md](../../10_ROS1基础/05_launch与参数服务器.md)

## Goals

- Understand the value of roslaunch over rosrun, and master the common launch file tags
- Distinguish private parameters (`~`) from global parameters, and load nested parameters in bulk from yaml files
- Be able to define launch arguments with `arg` that can be overridden on the command line
- Master the `rosparam get/set/list` command-line operations

Companion example package: [launch_param_demo](https://github.com/Romindly-Dev/romindly_ros1_tutorials/tree/main/launch_param_demo)

## How It Works

### Why roslaunch

Starting nodes with `rosrun` has two pain points:

1. **You must first manually run `roscore` in a terminal**; forget it and you get `Unable to communicate with master`
2. **One node occupies one terminal**, while a real robot often runs a dozen or more nodes simultaneously (drivers, TF, localization, navigation, ...) — typing them one by one is impractical

`roslaunch` solves both problems:

- It **automatically starts roscore** when it detects the master is not running
- One command **batch-starts a group of nodes** as described in XML, uniformly setting parameters, namespaces, and remappings
- When any node exits abnormally, it can be configured to restart automatically (`respawn`) or bring the whole launch down (`required`)

### The Parameter Server

The Parameter Server is a **global key-value dictionary** attached to the master. Nodes can have values written for them by launch before startup, and read/write them via API at run time. It is suited to **low-frequency configuration** (maximum speed, topic names, sensor mounting positions) — not to high-frequency data (that is what topics are for).

Parameters are organized by namespace:

| Notation | Meaning | Example |
|---|---|---|
| `/xxx` | Global parameter | `/use_sim_time` |
| `xxx` | Relative parameter (relative to the current namespace) | `robot1/max_speed` |
| `~xxx` | Node-private parameter (nested under the node name) | `/robot1/param_reader/max_speed` |

Private parameters are the most recommended practice: multiple instances of the same node do not interfere with each other, and `rosparam list` immediately shows which parameter belongs to whom.

## Running the Example

```bash
cd ~/ws_romindly && catkin_make && source devel/setup.bash
roslaunch launch_param_demo param_demo.launch
```

Expected output (excerpt; the second line notes that max_speed was overridden to 0.8 by the `<param>` in the launch file, and the last line lists this node's parameters):

```
[INFO] [...]: robot_name  = romindly_bot
[INFO] [...]: max_speed   = 0.80 m/s (被 launch 中 <param> 覆盖为 0.8)
[INFO] [...]: lidar_topic = /scan
[INFO] [...]: waypoints   = [[1.0, 0.0], [2.0, 1.0], [0.0, 2.0]]
[INFO] [...]: /use_sim_time = False
[INFO] [...]: 本节点参数列表: ['/robot1/param_reader/robot_name', '/robot1/param_reader/max_speed', ...]
```

Note two things: `max_speed` is 0.8 rather than the 1.2 in the yaml file (explained below); and all private parameters carry the `/robot1/param_reader/` prefix, because the node was placed in the `robot1` namespace.

Override the args on the command line and run again:

```bash
roslaunch launch_param_demo param_demo.launch use_sim:=true robot_ns:=robot2
```

Now `/use_sim_time = True`, the parameter prefix becomes `/robot2/param_reader/`, and an extra `sim_talker` node is started.

In another terminal, inspect and modify parameters with rosparam:

```bash
rosparam list                                   # list all parameters
rosparam get /robot1/param_reader/max_speed     # → 0.8
rosparam get /robot1/param_reader/sensor        # prints the whole nested tree as yaml
rosparam set /robot1/param_reader/max_speed 0.5 # modify at run time
```

Note: `rosparam set` only changes the value in the parameter server — **the node does not notice automatically**. Most nodes call `get_param` only once at startup. For dynamic run-time tuning, use `dynamic_reconfigure` (covered in a later chapter).

## Code Walkthrough

### param_demo.launch, Section by Section

File: [launch_param_demo/launch/param_demo.launch](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/launch_param_demo/launch/param_demo.launch)

**arg: the launch file's own startup arguments**

```xml
<arg name="use_sim" default="false"/>
<arg name="robot_ns" default="robot1"/>
```

An `arg` does not enter the parameter server; it takes effect only while the launch file is being parsed and is referenced with `$(arg xxx)`. Args with a `default` can be overridden on the command line with `name:=value`; if written as `value="..."` they cannot be overridden.

**param: writing a single parameter**

```xml
<param name="/use_sim_time" value="$(arg use_sim)"/>
```

A name starting with `/` writes a global parameter. `/use_sim_time` is a reserved ROS parameter controlling whether nodes use system time or simulated time; it will be central in the [08 rosbag chapter](08_rosbag_and_debugging.md).

**group + ns: namespace isolation**

```xml
<group ns="$(arg robot_ns)">
  <node pkg="launch_param_demo" type="param_reader.py" name="param_reader" output="screen">
    <rosparam file="$(find launch_param_demo)/config/robot_params.yaml" command="load"/>
    <param name="max_speed" value="0.8"/>
  </node>
</group>
```

- `group ns` prefixes all nodes, topics, and parameters inside it with `/robot1` — when multiple robots share one master, this is what avoids collisions
- The three essentials of `node`: `pkg` (package name), `type` (executable name; Python scripts include `.py`), `name` (the runtime node name, overriding the name in `init_node` in the code). `output="screen"` sends logs to the terminal instead of a log file
- **`<param>` / `<rosparam>` written inside a `<node>` automatically become that node's private parameters**. `rosparam file=... command="load"` loads the whole yaml file into the node's private namespace; the single `<param name="max_speed" value="0.8"/>` after it shares a name with the yaml entry, and **the later write overrides the earlier one** — that is where the 0.8 in the run output comes from. Typical usage: put defaults in yaml, override individual items in the launch file as needed
- `$(find package_name)` expands to the package's absolute path, keeping the launch file portable

**if: conditional startup**

```xml
<node pkg="rospy_tutorials" type="talker" name="sim_talker" if="$(arg use_sim)"/>
```

`if`/`unless` take a boolean and, combined with args, implement "one launch file for both simulation and real hardware".

Two more tags not used in this example but equally common:

- `<include file="$(find package_name)/launch/xxx.launch"/>`: nests another launch file; `<arg name=... value=.../>` passes arguments into the included file. Large systems are organized in layers through include
- `<remap from="scan" to="/front_lidar/scan"/>`: topic remapping, written inside a `<node>`, switching topic names without changing a single line of code

### robot_params.yaml and Nested Parameters

File: [launch_param_demo/config/robot_params.yaml](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/launch_param_demo/config/robot_params.yaml)

```yaml
max_speed: 1.2
sensor:
  lidar:
    topic: /scan
    frame_id: laser
waypoints: [[1.0, 0.0], [2.0, 1.0], [0.0, 2.0]]
```

The yaml indentation levels map directly to parameter namespace levels: `sensor.lidar.topic` becomes the parameter `~sensor/lidar/topic`. Lists (including nested lists) are stored whole as a single parameter.

### Reading Parameters in param_reader.py

File: [launch_param_demo/scripts/param_reader.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/launch_param_demo/scripts/param_reader.py)

```python
robot_name = rospy.get_param("~robot_name", "unknown")
max_speed = rospy.get_param("~max_speed", 0.5)
lidar_topic = rospy.get_param("~sensor/lidar/topic", "/scan")
```

- The `~` prefix = private parameter; the second argument is the **default value used when the parameter is missing** — production code should always provide defaults, otherwise a missing parameter raises `KeyError` and crashes the node
- Nested parameters are indexed level by level with `/`; alternatively, `rospy.get_param("~sensor")` retrieves the whole dictionary at once for you to parse yourself

```python
use_sim_time = rospy.get_param("/use_sim_time", False)   # global parameter
rospy.set_param("~runtime_flag", True)                    # write a parameter at run time
```

Every call to `get_param`/`set_param` incurs a network round trip to the master — **do not put them in high-frequency loops**; read once at startup and store in a member variable.

## Hands-on Exercises

1. Add a `camera` section under `sensor` in `robot_params.yaml` (with `topic: /image_raw` and `frame_id: camera_link`), and modify `param_reader.py` to read and print `~sensor/camera/topic`. Verify that `rosparam get /robot1/param_reader/sensor/camera` shows the new parameters.
2. Add `<arg name="speed_override" default="0.8"/>` to `param_demo.launch`, change the value of the node-level `<param name="max_speed" value="0.8"/>` to `$(arg speed_override)`, then verify with `roslaunch launch_param_demo param_demo.launch speed_override:=2.0` that the output becomes 2.0.

## FAQ

**Q: `roslaunch` fails with `cannot launch node of type [launch_param_demo/param_reader.py]`?**
The Python script lacks execute permission. Run `chmod +x ~/ws_romindly/src/romindly_ros1_tutorials/launch_param_demo/scripts/param_reader.py`. Also confirm the script's first line is `#!/usr/bin/env python3`.

**Q: I changed a yaml parameter and re-ran `rosparam set`, but the node's behavior did not change?**
Nodes read parameters only once at startup. After changing the yaml you must re-run roslaunch; for run-time tuning use `dynamic_reconfigure`.

**Q: The parameter prefixes in `rosparam list` don't match my expectations?**
Check the three stacked namespace layers: `group ns` > node name > parameter name. The full path of a private parameter is `/<ns>/<node_name>/<param>`. Use `rosparam list | grep keyword` to locate things quickly.

**Q: What is the difference between arg and param?**
An `arg` is a "macro variable" of the launch-file parsing phase; it never enters the parameter server and nodes cannot read it. Only `param` actually writes to the parameter server. The common combination is `<param name="x" value="$(arg x)"/>` to turn an arg into a param.
