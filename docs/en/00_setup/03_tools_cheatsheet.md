# Common Tools Cheat Sheet

> 中文版 / Chinese: [03_常用工具速查.md](../../00_环境搭建/03_常用工具速查.md)

## Goal

A compact reference of the command-line and graphical tools most frequently used for day-to-day ROS1 debugging — one-line description plus an example for each. Read it once end to end, then use it as a lookup manual afterwards.

## Background

ROS1 debugging revolves around three kinds of objects: nodes, topics, and services, plus parameters (param) and message/service type definitions (msg/srv). Each kind of object has a corresponding `ros*` command-line tool; on the graphical side, the rqt family and RViz are the mainstays. All commands below require the environment to be sourced and roscore (or some launch) to be running.

## Command-Line Tools

### rostopic — topics

| Command | Description | Example |
| --- | --- | --- |
| `rostopic list` | List all current topics | `rostopic list` |
| `rostopic echo` | Print topic content | `rostopic echo /odom` |
| `rostopic hz` | Measure topic publish rate | `rostopic hz /scan` |
| `rostopic info` | Show topic type and publishers/subscribers | `rostopic info /cmd_vel` |
| `rostopic type` | Show only the message type | `rostopic type /cmd_vel` |
| `rostopic pub` | Manually publish a message | `rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}'` |
| `rostopic bw` | Measure topic bandwidth usage | `rostopic bw /camera/image_raw` |

### rosnode — nodes

| Command | Description | Example |
| --- | --- | --- |
| `rosnode list` | List all running nodes | `rosnode list` |
| `rosnode info` | Show a node's topic/service connections | `rosnode info /move_base` |
| `rosnode ping` | Test whether a node is alive | `rosnode ping /amcl` |
| `rosnode kill` | Kill a specific node | `rosnode kill /rviz` |
| `rosnode cleanup` | Clean up dead but unregistered nodes | `rosnode cleanup` |

### rosservice — services

| Command | Description | Example |
| --- | --- | --- |
| `rosservice list` | List all services | `rosservice list` |
| `rosservice info` | Show service type and providing node | `rosservice info /global_localization` |
| `rosservice type` | Show only the service type | `rosservice type /clear` |
| `rosservice call` | Call a service | `rosservice call /global_localization "{}"` |
| `rosservice args` | Show the request argument format of a service | `rosservice args /spawn` |

### rosparam — parameters

| Command | Description | Example |
| --- | --- | --- |
| `rosparam list` | List all parameters on the parameter server | `rosparam list` |
| `rosparam get` | Read a parameter value | `rosparam get /move_base/base_local_planner` |
| `rosparam set` | Set a parameter value | `rosparam set /amcl/min_particles 500` |
| `rosparam dump` | Export parameters to YAML | `rosparam dump params.yaml /move_base` |
| `rosparam load` | Load parameters from YAML | `rosparam load params.yaml /move_base` |
| `rosparam delete` | Delete a parameter | `rosparam delete /foo` |

### rosmsg / rossrv — type definitions

| Command | Description | Example |
| --- | --- | --- |
| `rosmsg show` | Show message field definitions | `rosmsg show sensor_msgs/LaserScan` |
| `rosmsg list` | List all message types | `rosmsg list \| grep nav_msgs` |
| `rosmsg package` | List messages defined by a package | `rosmsg package geometry_msgs` |
| `rossrv show` | Show a service definition (request above `---`, response below) | `rossrv show nav_msgs/GetMap` |
| `rossrv list` | List all service types | `rossrv list \| grep std_srvs` |

## Graphical and Diagnostic Tools

| Tool | Description | Launch command |
| --- | --- | --- |
| rqt_graph | Visualize the node–topic connection graph; the first choice for figuring out "who is not connected to whom" | `rqt_graph` |
| rqt_console | Centralized view of all node logs, filterable by level/node | `rqt_console` |
| rviz | 3D visualization: TF, point clouds, maps, paths, models, etc. | `rviz` |
| tf2_tools | Generate a PDF of the current TF tree (frames.pdf) to diagnose broken frame chains | `rosrun tf2_tools view_frames.py`, then `evince frames.pdf` |
| roswtf | One-shot health check: environment, graph connectivity, TF, and other common issues | `roswtf` |

> Tip: `rosrun tf2_tools view_frames.py` carries the `.py` suffix in Noetic; older tutorials that write `view_frames` are for Melodic and earlier.

## catkin_make vs catkin build

| | catkin_make | catkin build |
| --- | --- | --- |
| Source | Ships with ROS | catkin_tools package, installed separately |
| Build model | Whole workspace as one CMake project | Each package built independently in an isolated environment |
| Single-package builds | `catkin_make --only-pkg-with-deps <pkg>` (clumsy, leaves residual state) | `catkin build <pkg>` (native support) |
| Error localization | Mixed together, hard to read | Output grouped per package, clear |
| Best suited for | Small workspaces, following official tutorials | Real projects with many packages (recommended) |

Install catkin build:

```bash
sudo apt install -y python3-catkin-tools
```

Common commands:

```bash
cd ~/ws_romindly
catkin build          # build everything
catkin build urdf_demo  # build only the given package and its dependencies
catkin clean          # clean build/devel/logs
```

**Note**: the two tools must not be mixed in the same workspace. If you previously used `catkin_make`, delete `build/` and `devel/` before switching (or run `catkin clean`). The documentation of this kit uses `catkin_make` by default — pick one and stick with it.

## Troubleshooting

- `rostopic echo` shows no output but `rostopic info` shows a publisher: usually a message type mismatch or a TF/sim-time issue; first run `rostopic hz` to confirm data is actually being published.
- Every command in multiple terminals reports `Unable to communicate with master`: roscore is not running, or `ROS_MASTER_URI` points at an unreachable address (common in multi-machine setups).
- rqt windows won't open (over SSH): a graphical environment is required; use `ssh -X` or run on the local desktop.
- `rosmsg show` cannot find a custom message: the package defining the message has not been built, or `devel/setup.bash` has not been sourced.
