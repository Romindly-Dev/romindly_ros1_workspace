# 08 rosbag 与调试工具

> English version: [08_rosbag_and_debugging.md](../en/10_ros1_basics/08_rosbag_and_debugging.md)

## 目标

- 掌握 `rosbag record / info / play` 的常用参数，完成一次录制与回放
- 理解 `use_sim_time` 与 `--clock` 的配合机制——这是后续**离线建图教程的关键前置知识**
- 会用 `rqt_bag` 可视化检查数据包
- 建立一套节点/话题问题的排查工具箱：`rqt_console`、`rosnode ping`、`roswtf`、`rostopic delay/bw`

演示数据来源：[topic_demo](https://github.com/Romindly-Dev/romindly_ros1_tutorials/tree/main/topic_demo) 的 [talker.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/topic_demo/scripts/talker.py)（以 10Hz 向 `/robot_status` 发布 [romindly_msgs/RobotStatus](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/romindly_msgs/msg/RobotStatus.msg) 消息）。

## 原理简介

rosbag 把话题消息**连同时间戳**原样序列化到 `.bag` 文件，回放时按原始时序重新发布。价值在于：

- **现场数据带回实验室**：在客户现场录一段传感器数据，回办公室反复调算法，不用每次都跑真机
- **可复现**：同一份 bag 回放一百次，输入完全一致，便于对比参数效果
- **离线建图**：先录激光+里程计+TF，再回放喂给建图算法（gmapping/cartographer），是 2D 建图章节的标准工作流

回放有个关键问题：算法节点里的 `rospy.Time.now()` 该返回什么时间？bag 里的数据是"过去"录的，如果节点用系统当前时间，TF 查询、消息同步全会错乱。ROS 的解法是**仿真时钟**：设置全局参数 `/use_sim_time = true` 后，所有节点的时间源改为订阅 `/clock` 话题；`rosbag play --clock` 负责按 bag 内的时间戳发布 `/clock`。两者**必须成对使用**，只开一半就会出现"时间静止"或"时间错乱"。

## 运行示例

### 录制

```bash
cd ~/ws_romindly && catkin_make && source devel/setup.bash
roscore
```

另开终端启动数据源并录制：

```bash
# 终端 2：启动发布者
rosrun topic_demo talker.py

# 终端 3：只录 /robot_status，指定输出文件名，录 30 秒后自动停止
cd ~/ws_romindly
rosbag record /robot_status -O robot_status_demo --duration=30
```

`record` 常用参数：

| 参数 | 作用 |
|---|---|
| `/topic1 /topic2 ...` | 按话题名录制（推荐，bag 体积小） |
| `-a` | 录**所有**话题——图省事但真机上传感器数据量大，慎用 |
| `-O 名字` | 指定输出文件名（默认是时间戳命名） |
| `--duration=30` | 录 30 秒自动停；也支持 `--duration=5m` |
| `-e "/robot.*"` | 正则匹配话题 |

预期输出：

```
[ INFO] [...]: Subscribing to /robot_status
[ INFO] [...]: Recording to 'robot_status_demo.bag'.
```

30 秒后自动退出。查看 bag 概要：

```bash
rosbag info robot_status_demo.bag
```

预期输出（消息数约 300 条 = 10Hz × 30s）：

```
path:        robot_status_demo.bag
duration:    30.0s
messages:    300
topics:      /robot_status   300 msgs    : romindly_msgs/RobotStatus
```

### 回放

先**停掉 talker.py**（Ctrl+C，否则实时数据和回放数据混在一起），保留 roscore：

```bash
# 终端 2：订阅端等着收
rosrun topic_demo listener.py

# 终端 3：回放
rosbag play robot_status_demo.bag
```

listener 打印出录制时的电量/里程数据，和实时运行时一样：

```
[INFO] [...]: [romindly_bot] state=1 battery=100.0% mileage=0.05m
[INFO] [...]: [romindly_bot] state=1 battery=100.0% mileage=0.10m
```

`play` 常用参数：

```bash
rosbag play robot_status_demo.bag -r 2      # 2 倍速回放
rosbag play robot_status_demo.bag --pause   # 启动即暂停，按空格逐步播放
rosbag play robot_status_demo.bag -l        # 循环回放
```

### use_sim_time + --clock（离线建图的标准姿势）

```bash
# 必须在启动任何算法节点之前设置！
rosparam set /use_sim_time true

# 回放时发布 /clock
rosbag play robot_status_demo.bag --clock
```

验证：`rostopic echo /clock` 能看到时间在走，且数值是**录制时刻**的时间。后续离线建图章节的流程就是：`rosparam set /use_sim_time true` → 启动 gmapping → `rosbag play xxx.bag --clock`，顺序不能乱——`use_sim_time` 必须在算法节点启动**之前**生效，节点只在初始化时决定时间源。用完记得 `rosparam set /use_sim_time false`，否则之后跑实时节点会卡在等 `/clock`。

### rqt_bag 可视化

```bash
rqt_bag robot_status_demo.bag
```

时间轴上每个小竖线是一条消息；右键话题 → View → Plot 可以把 `battery`、`mileage` 字段画成曲线，右键 → Publish 可以只回放选中话题。检查"传感器是否断流、频率是否稳定"用它最直观。

## 调试工具合集

### rqt_console：日志分级过滤

```bash
rqt_console
```

集中显示所有节点的日志，可按级别（Debug/Info/Warn/Error/Fatal）、节点名、关键词过滤。比在十几个终端里翻滚动日志高效得多。配合 `rqt_logger_level` 可以在线把某个节点的日志级别调到 Debug，不用重启节点。

### rosnode ping：节点还活着吗

```bash
rosnode ping /listener_py
# → xmlrpc reply from http://...  time=0.83ms
```

有回复说明节点进程存活且 XML-RPC 接口正常；`unable to contact` 但 `rosnode list` 里还有它，说明节点已死但没注销——用 `rosnode cleanup` 清理僵尸注册信息。

### roswtf：系统体检

```bash
roswtf
```

一键检查环境变量、包依赖、节点连接图、TF 等常见问题，输出 WARNING/ERROR 列表。典型能查出：话题发布者和订阅者**类型不匹配**、订阅了没人发布的话题、机器间时间不同步。排查"莫名收不到消息"时先跑它。

### rostopic delay / bw：延迟与带宽

```bash
rostopic delay /robot_status   # 消息 header.stamp 与到达时间之差
rostopic bw /robot_status      # 话题占用带宽
rostopic hz /robot_status      # 实际频率（回顾）
```

`delay` 要求消息带 Header（RobotStatus 有），是排查"传感器数据延迟大"的直接手段；`bw` 用于评估录包体积和网络负载——录 `-a` 之前先用它看看相机话题有多少 MB/s。

## 动手练习

1. 把 [talker.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/topic_demo/scripts/talker.py) 中电量衰减量从 `0.01` 改为 `1.0`，重新录 30 秒并回放给 listener.py，验证回放到后半段时触发"电量过低"WARN；再用 `rqt_bag` 把 `battery` 字段画成曲线观察下降斜率。
2. 用 `rosbag play -r 4` 四倍速回放同一个 bag，用 `rostopic hz /robot_status` 验证频率变为约 40Hz；思考：`header.stamp` 里的时间戳会跟着变吗？（用 `rostopic echo /robot_status/header` 验证，再试试加 `--clock` 后 `rostopic delay` 的读数变化。）

## 常见问题

**Q: 回放时 listener 收不到消息？**
依次检查：roscore 是否在跑（`rosbag play` 不会自动起 master）；bag 里的话题名与订阅名是否一致（`rosbag info` 核对）；是否设置了 `/use_sim_time true` 却没加 `--clock`——此时节点等不到时钟，`rospy.Rate`、定时器全部停摆，看起来就是"收不到"。

**Q: 设了 use_sim_time 后连 `rostopic echo` 都不动了？**
同上，仿真时间模式下没有 `/clock` 发布者时时间为 0。要么开始回放（带 `--clock`），要么 `rosparam set /use_sim_time false` 恢复。

**Q: 回放带 TF 的 bag 时报 ExtrapolationException？**
录制时的 TF 时间戳是过去时刻，必须用 `--clock` + `use_sim_time` 让全系统进入 bag 时间。若 bag 里已录了 `/tf_static` 之外又有节点在发同名 TF，还会时间跳变——回放前停掉所有实时 TF 广播节点。

**Q: bag 文件太大怎么办？**
录制时按话题录而不是 `-a`；事后用 `rosbag filter in.bag out.bag "topic == '/robot_status'"` 抽取子集，或 `rosbag compress` 压缩。

**Q: `rosbag record` 提示 buffer 溢出丢消息？**
磁盘写入跟不上。换 SSD 路径、减少话题、或加 `-b 1024` 增大缓冲（MB）。
