# 08 rosbag and Debugging Tools

> 中文版 / Chinese: [08_rosbag与调试工具.md](../../10_ROS1基础/08_rosbag与调试工具.md)

## Goals

- Master the common options of `rosbag record / info / play` and complete a record-and-playback cycle
- Understand how `use_sim_time` and `--clock` work together — **key prerequisite knowledge for the later offline mapping tutorials**
- Be able to inspect bag files visually with `rqt_bag`
- Build a troubleshooting toolbox for node/topic problems: `rqt_console`, `rosnode ping`, `roswtf`, `rostopic delay/bw`

Demo data source: [talker.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/topic_demo/scripts/talker.py) from [topic_demo](https://github.com/Romindly-Dev/romindly_ros1_tutorials/tree/main/topic_demo) (publishing [romindly_msgs/RobotStatus](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/romindly_msgs/msg/RobotStatus.msg) messages to `/robot_status` at 10 Hz).

## How It Works

rosbag serializes topic messages **together with their timestamps** into a `.bag` file, and playback republishes them in the original timing. Its value:

- **Bring field data back to the lab**: record a stretch of sensor data at a customer site, then tune your algorithm repeatedly back at the office without re-running the real robot every time
- **Reproducibility**: play the same bag a hundred times and the input is identical every time, making parameter comparisons easy
- **Offline mapping**: record laser + odometry + TF first, then play it back into a mapping algorithm (gmapping/cartographer) — the standard workflow of the 2D mapping chapters

Playback raises a key question: what time should `rospy.Time.now()` return inside an algorithm node? The data in the bag was recorded "in the past"; if nodes use the current system time, TF lookups and message synchronization all break. ROS's solution is the **simulated clock**: after setting the global parameter `/use_sim_time = true`, all nodes switch their time source to the `/clock` topic; `rosbag play --clock` publishes `/clock` according to the timestamps inside the bag. The two **must be used as a pair** — enabling only one produces "frozen time" or "scrambled time".

## Running the Example

### Recording

```bash
cd ~/ws_romindly && catkin_make && source devel/setup.bash
roscore
```

Start the data source and record in other terminals:

```bash
# Terminal 2: start the publisher
rosrun topic_demo talker.py

# Terminal 3: record only /robot_status, name the output file, stop automatically after 30 seconds
cd ~/ws_romindly
rosbag record /robot_status -O robot_status_demo --duration=30
```

Common `record` options:

| Option | Effect |
|---|---|
| `/topic1 /topic2 ...` | Record by topic name (recommended; keeps the bag small) |
| `-a` | Record **all** topics — convenient, but on a real robot sensor data volume is huge; use with caution |
| `-O name` | Name the output file (default is a timestamp-based name) |
| `--duration=30` | Stop automatically after 30 seconds; `--duration=5m` also works |
| `-e "/robot.*"` | Match topics by regular expression |

Expected output:

```
[ INFO] [...]: Subscribing to /robot_status
[ INFO] [...]: Recording to 'robot_status_demo.bag'.
```

It exits automatically after 30 seconds. View the bag summary:

```bash
rosbag info robot_status_demo.bag
```

Expected output (about 300 messages = 10 Hz × 30 s):

```
path:        robot_status_demo.bag
duration:    30.0s
messages:    300
topics:      /robot_status   300 msgs    : romindly_msgs/RobotStatus
```

### Playback

First **stop talker.py** (Ctrl+C — otherwise live data and playback data get mixed together), keeping roscore running:

```bash
# Terminal 2: subscriber waiting to receive
rosrun topic_demo listener.py

# Terminal 3: play back
rosbag play robot_status_demo.bag
```

The listener prints the battery/mileage data from recording time, exactly as when running live:

```
[INFO] [...]: [romindly_bot] state=1 battery=100.0% mileage=0.05m
[INFO] [...]: [romindly_bot] state=1 battery=100.0% mileage=0.10m
```

Common `play` options:

```bash
rosbag play robot_status_demo.bag -r 2      # play back at 2x speed
rosbag play robot_status_demo.bag --pause   # start paused, step through with the space bar
rosbag play robot_status_demo.bag -l        # loop playback
```

### use_sim_time + --clock (the Standard Recipe for Offline Mapping)

```bash
# Must be set BEFORE starting any algorithm node!
rosparam set /use_sim_time true

# Publish /clock during playback
rosbag play robot_status_demo.bag --clock
```

Verification: `rostopic echo /clock` shows time advancing, with values from the **recording time**. The later offline mapping chapters follow exactly this flow: `rosparam set /use_sim_time true` → start gmapping → `rosbag play xxx.bag --clock`. The order matters — `use_sim_time` must take effect **before** the algorithm nodes start, since a node decides its time source only at initialization. Remember to `rosparam set /use_sim_time false` when done, or live nodes launched afterwards will hang waiting for `/clock`.

### Visualizing with rqt_bag

```bash
rqt_bag robot_status_demo.bag
```

Each small tick on the timeline is one message. Right-click a topic → View → Plot to graph the `battery` and `mileage` fields as curves; right-click → Publish replays only the selected topics. It is the most intuitive way to check "did the sensor stream drop out, is the rate stable".

## Debugging Tool Collection

### rqt_console: Log Filtering by Level

```bash
rqt_console
```

Shows all nodes' logs in one place, filterable by level (Debug/Info/Warn/Error/Fatal), node name, or keyword. Far more efficient than scrolling logs across a dozen terminals. Combined with `rqt_logger_level`, you can raise a node's log level to Debug online without restarting it.

### rosnode ping: Is the Node Still Alive

```bash
rosnode ping /listener_py
# → xmlrpc reply from http://...  time=0.83ms
```

A reply means the node process is alive and its XML-RPC interface works. `unable to contact` while the node still appears in `rosnode list` means the node died without deregistering — clean up the zombie registration with `rosnode cleanup`.

### roswtf: System Health Check

```bash
roswtf
```

One command checks environment variables, package dependencies, the node connection graph, TF, and other common problems, outputting a WARNING/ERROR list. Typical catches: **type mismatch** between a topic's publisher and subscriber, subscribing to a topic nobody publishes, clock skew between machines. Run it first when chasing a mysterious "messages not arriving" problem.

### rostopic delay / bw: Latency and Bandwidth

```bash
rostopic delay /robot_status   # difference between the message header.stamp and arrival time
rostopic bw /robot_status      # bandwidth consumed by the topic
rostopic hz /robot_status      # actual rate (recap)
```

`delay` requires messages with a Header (RobotStatus has one) and is the direct tool for diagnosing "sensor data latency is high"; `bw` evaluates bag size and network load — before recording with `-a`, check how many MB/s the camera topics produce.

## Hands-on Exercises

1. Change the battery drain in [talker.py](https://github.com/Romindly-Dev/romindly_ros1_tutorials/blob/main/topic_demo/scripts/talker.py) from `0.01` to `1.0`, re-record 30 seconds, and play it back into listener.py; verify that the "battery low" WARN triggers in the second half of playback. Then use `rqt_bag` to plot the `battery` field and observe the slope of the decline.
2. Play the same bag at 4x with `rosbag play -r 4` and verify with `rostopic hz /robot_status` that the rate becomes about 40 Hz. Consider: does the timestamp in `header.stamp` change accordingly? (Verify with `rostopic echo /robot_status/header`, then try adding `--clock` and watch how the `rostopic delay` reading changes.)

## FAQ

**Q: The listener receives nothing during playback?**
Check in order: is roscore running (`rosbag play` does not start a master automatically); do the topic names in the bag match the subscription (verify with `rosbag info`); did you set `/use_sim_time true` without adding `--clock` — in that case nodes wait for a clock that never comes, `rospy.Rate` and timers all stall, and it looks like "nothing is received".

**Q: After setting use_sim_time, even `rostopic echo` freezes?**
Same as above: in simulated-time mode, time is 0 when no `/clock` publisher exists. Either start playback (with `--clock`) or restore with `rosparam set /use_sim_time false`.

**Q: Playing a bag containing TF throws ExtrapolationException?**
The TF timestamps in the recording are in the past; you must use `--clock` + `use_sim_time` to put the whole system on bag time. If, besides the `/tf_static` in the bag, some node is also publishing the same TF live, you will additionally get time jumps — stop all live TF broadcasters before playback.

**Q: The bag file is too large — what to do?**
Record by topic instead of `-a`; afterwards extract a subset with `rosbag filter in.bag out.bag "topic == '/robot_status'"`, or compress with `rosbag compress`.

**Q: `rosbag record` warns about buffer overflow and dropped messages?**
Disk writes cannot keep up. Use an SSD path, reduce the topics, or enlarge the buffer with `-b 1024` (MB).
