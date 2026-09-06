# 05 launch 与参数服务器

## 目标

- 理解 roslaunch 相比 rosrun 的价值，掌握 launch 文件的常用标签
- 区分私有参数（`~`）与全局参数，会用 yaml 文件批量加载嵌套参数
- 会用 `arg` 定义可在命令行覆盖的启动参数
- 掌握 `rosparam get/set/list` 命令行操作

配套示例包：[launch_param_demo](https://github.com/Romindly-Dev/romindly_ros1_tutorials/tree/main/launch_param_demo)

## 原理简介

### 为什么用 roslaunch

用 `rosrun` 启动节点有两个痛点：

1. **必须先手动开一个终端跑 `roscore`**，忘了就报 `Unable to communicate with master`
2. **一个节点占一个终端**，真实机器人往往要同时跑十几个节点（驱动、TF、定位、导航……），逐个手敲不现实

`roslaunch` 解决了这两个问题：

- 检测到 master 未运行时**自动启动 roscore**
- 一条命令按 XML 描述**批量启动一组节点**，并统一设置参数、命名空间、重映射
- 任一节点异常退出时可配置自动重启（`respawn`）或整体退出（`required`）

### 参数服务器

参数服务器（Parameter Server）是挂在 master 上的一个**全局键值字典**，节点可以在启动前由 launch 写入、运行时用 API 读写。它适合存放**低频配置**（最大速度、话题名、传感器安装位置），不适合传高频数据（那是话题的事）。

参数按命名空间组织：

| 写法 | 含义 | 例子 |
|---|---|---|
| `/xxx` | 全局参数 | `/use_sim_time` |
| `xxx` | 相对参数（相对当前命名空间） | `robot1/max_speed` |
| `~xxx` | 节点私有参数（挂在节点名下） | `/robot1/param_reader/max_speed` |

私有参数是最推荐的做法：同一个节点起多份实例互不干扰，`rosparam list` 时一眼能看出参数属于谁。

## 运行示例

```bash
cd ~/ws_romindly && catkin_make && source devel/setup.bash
roslaunch launch_param_demo param_demo.launch
```

预期输出（节选）：

```
[INFO] [...]: robot_name  = romindly_bot
[INFO] [...]: max_speed   = 0.80 m/s (被 launch 中 <param> 覆盖为 0.8)
[INFO] [...]: lidar_topic = /scan
[INFO] [...]: waypoints   = [[1.0, 0.0], [2.0, 1.0], [0.0, 2.0]]
[INFO] [...]: /use_sim_time = False
[INFO] [...]: 本节点参数列表: ['/robot1/param_reader/robot_name', '/robot1/param_reader/max_speed', ...]
```

注意两点：`max_speed` 是 0.8 而不是 yaml 里的 1.2（后面讲为什么）；所有私有参数都带 `/robot1/param_reader/` 前缀，因为节点被放进了 `robot1` 命名空间。

命令行覆盖 arg，再跑一次：

```bash
roslaunch launch_param_demo param_demo.launch use_sim:=true robot_ns:=robot2
```

此时 `/use_sim_time = True`，参数前缀变为 `/robot2/param_reader/`，并且会额外启动一个 `sim_talker` 节点。

另开终端，用 rosparam 查看和修改参数：

```bash
rosparam list                                   # 列出所有参数
rosparam get /robot1/param_reader/max_speed     # → 0.8
rosparam get /robot1/param_reader/sensor        # 整棵嵌套树以 yaml 打印
rosparam set /robot1/param_reader/max_speed 0.5 # 运行时修改
```

注意：`rosparam set` 只改参数服务器里的值，**节点不会自动感知**——大多数节点只在启动时 `get_param` 一次。需要运行时动态调参请用 `dynamic_reconfigure`（后续章节）。

## 代码讲解

### param_demo.launch 逐段解析

文件：[launch_param_demo/launch/param_demo.launch](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/launch_param_demo/launch/param_demo.launch)

**arg：launch 文件自身的启动参数**

```xml
<arg name="use_sim" default="false"/>
<arg name="robot_ns" default="robot1"/>
```

`arg` 不进参数服务器，只在 launch 解析阶段生效，用 `$(arg xxx)` 引用。有 `default` 的可以在命令行用 `名字:=值` 覆盖；如果写成 `value="..."` 则不可覆盖。

**param：写入单个参数**

```xml
<param name="/use_sim_time" value="$(arg use_sim)"/>
```

名字以 `/` 开头表示写全局参数。`/use_sim_time` 是 ROS 的保留参数，控制节点用系统时间还是仿真时间，在 [08 rosbag 章节](08_rosbag与调试工具.md) 会重点用到。

**group + ns：命名空间隔离**

```xml
<group ns="$(arg robot_ns)">
  <node pkg="launch_param_demo" type="param_reader.py" name="param_reader" output="screen">
    <rosparam file="$(find launch_param_demo)/config/robot_params.yaml" command="load"/>
    <param name="max_speed" value="0.8"/>
  </node>
</group>
```

- `group ns` 把内部所有节点、话题、参数都套上 `/robot1` 前缀——多机器人共用一个 master 时靠它避免冲突
- `node` 三要素：`pkg`（包名）、`type`（可执行文件名，Python 脚本带 `.py`）、`name`（运行时节点名，覆盖代码里 `init_node` 的名字）。`output="screen"` 让日志打到终端而非日志文件
- **写在 `<node>` 内部的 `<param>` / `<rosparam>` 自动成为该节点的私有参数**。`rosparam file=... command="load"` 把整个 yaml 载入节点私有命名空间；后面的单个 `<param name="max_speed" value="0.8"/>` 与 yaml 中同名，**后写的覆盖先写的**——这就是运行结果里 0.8 的来源。典型用法：yaml 放默认值，launch 里按需覆盖个别项
- `$(find 包名)` 展开为包的绝对路径，保证 launch 文件可移植

**if：条件启动**

```xml
<node pkg="rospy_tutorials" type="talker" name="sim_talker" if="$(arg use_sim)"/>
```

`if`/`unless` 接布尔值，配合 arg 实现"仿真/实机一套 launch"。

本示例未用到但同样常用的两个标签：

- `<include file="$(find 包名)/launch/xxx.launch"/>`：嵌套引用其他 launch，可用 `<arg name=... value=.../>` 向被引用文件传参，大型系统都靠 include 分层组织
- `<remap from="scan" to="/front_lidar/scan"/>`：话题重映射，写在 `<node>` 内部，让代码不改一行就换话题名

### robot_params.yaml 与嵌套参数

文件：[launch_param_demo/config/robot_params.yaml](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/launch_param_demo/config/robot_params.yaml)

```yaml
max_speed: 1.2
sensor:
  lidar:
    topic: /scan
    frame_id: laser
waypoints: [[1.0, 0.0], [2.0, 1.0], [0.0, 2.0]]
```

yaml 的缩进层级直接映射为参数命名空间层级：`sensor.lidar.topic` 变成参数 `~sensor/lidar/topic`。列表（包括嵌套列表）会整体存为一个参数。

### param_reader.py 读取参数

文件：[launch_param_demo/scripts/param_reader.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/launch_param_demo/scripts/param_reader.py)

```python
robot_name = rospy.get_param("~robot_name", "unknown")
max_speed = rospy.get_param("~max_speed", 0.5)
lidar_topic = rospy.get_param("~sensor/lidar/topic", "/scan")
```

- `~` 前缀 = 私有参数；第二个实参是**取不到时的默认值**——生产代码务必给默认值，否则参数缺失时抛 `KeyError` 直接崩节点
- 嵌套参数用 `/` 逐级索引；也可以 `rospy.get_param("~sensor")` 一次取回整个字典自己解析

```python
use_sim_time = rospy.get_param("/use_sim_time", False)   # 全局参数
rospy.set_param("~runtime_flag", True)                    # 运行时写参数
```

`get_param`/`set_param` 每次调用都要走一次与 master 的网络请求，**不要放在高频循环里**，启动时读一次存成员变量即可。

## 动手练习

1. 在 `robot_params.yaml` 的 `sensor` 下新增 `camera` 一节（含 `topic: /image_raw` 和 `frame_id: camera_link`），并修改 `param_reader.py` 读取并打印 `~sensor/camera/topic`。验证 `rosparam get /robot1/param_reader/sensor/camera` 能看到新参数。
2. 给 `param_demo.launch` 新增 `<arg name="speed_override" default="0.8"/>`，把节点内 `<param name="max_speed" value="0.8"/>` 的 value 改为 `$(arg speed_override)`，然后用 `roslaunch launch_param_demo param_demo.launch speed_override:=2.0` 验证输出变为 2.0。

## 常见问题

**Q: `roslaunch` 报 `cannot launch node of type [launch_param_demo/param_reader.py]`？**
Python 脚本没有可执行权限。执行 `chmod +x ~/ws_romindly/src/romindly_ros1_tutorials/launch_param_demo/scripts/param_reader.py`。另外确认脚本首行有 `#!/usr/bin/env python3`。

**Q: 改了 yaml 参数，重新 `rosparam set` 后节点行为没变？**
节点只在启动时读一次参数。改 yaml 后需要重新 roslaunch；运行时调参用 `dynamic_reconfigure`。

**Q: `rosparam list` 里参数前缀和预期不一致？**
检查三层命名空间叠加：`group ns` > 节点名 > 参数名。私有参数完整路径是 `/<ns>/<node_name>/<param>`。用 `rosparam list | grep 关键词` 快速定位。

**Q: arg 和 param 什么区别？**
`arg` 是 launch 文件解析期的"宏变量"，不进参数服务器，节点读不到；`param` 才真正写入参数服务器。常见组合是 `<param name="x" value="$(arg x)"/>` 把 arg 转成 param。
