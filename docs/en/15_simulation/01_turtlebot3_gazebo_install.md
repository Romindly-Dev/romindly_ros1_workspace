# 01 Installing TurtleBot3 and Gazebo

> 中文版 / Chinese: [01_TurtleBot3与Gazebo安装.md](../../15_仿真入门/01_TurtleBot3与Gazebo安装.md)

## Goal

- Confirm Gazebo 11 was installed with desktop-full and starts correctly
- Install the TurtleBot3 trio (simulation, messages, teleop) via apt
- Understand the role of the `TURTLEBOT3_MODEL` environment variable and write it into `~/.bashrc`
- Launch your first simulation world, `turtlebot3_world.launch`

## How It Works

**Gazebo** is the standard physics simulator of the ROS1 ecosystem: it simulates rigid-body dynamics, sensors (LiDAR, IMU, cameras), and actuators, and publishes the simulated data as ROS topics/TF through the `gazebo_ros` bridge packages. To higher-level algorithms (mapping, navigation), **the topic interfaces published by the simulated robot and the physical robot are exactly the same** — which is why the "simulate first, then go real" approach works.

**TurtleBot3** (TB3 below) is an open-source educational robot by ROBOTIS and the de facto standard practice platform in the ROS community; complete Gazebo models plus mapping and navigation configurations are provided officially. It consists of three package groups:

| Repository | Content |
| --- | --- |
| `turtlebot3` | Robot description (URDF), bringup, teleop, navigation configuration |
| `turtlebot3_msgs` | TB3-specific message definitions |
| `turtlebot3_simulations` | Gazebo worlds, simulation models, and plugin configuration |

The organization has forked these three repositories (noetic branch) and included them in the `vcs import` manifest, but **for the learning phase, installing directly via apt is recommended** — it avoids compilation, and the versions have been validated through ROBOTIS's official releases.

## Step-by-Step Instructions

### Step 1: Verify Gazebo

The `ros-noetic-desktop-full` installed in chapter 00 already ships with Gazebo 11; confirm first:

```bash
gazebo --version
```

Expected output:

```
Gazebo multi-robot simulator, version 11.15.1
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org
```

Any 11.x version is fine (the minor version varies with updates). If you get `command not found`, desktop-full is not what you installed; fix it with `sudo apt install ros-noetic-desktop-full`.

You can start it bare for a quick look (an empty world + ground plane + light source; close it when done):

```bash
gazebo
```

### Step 2: Install TurtleBot3 (apt, recommended)

```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-teleop
```

`ros-noetic-turtlebot3` is a metapackage that automatically pulls in `turtlebot3-msgs`, `turtlebot3-description`, and other dependencies. Verify:

```bash
rospack find turtlebot3_gazebo
```

Expected output:

```
/opt/ros/noetic/share/turtlebot3_gazebo
```

> **Alternative: source installation.** If you later need to modify the TB3 models or plugin parameters, use the source route: the `vcs import` manifest from chapter 00 already includes the organization's forks of `turtlebot3` / `turtlebot3_msgs` / `turtlebot3_simulations` (noetic branch); after pulling, just run `catkin_make` under `~/ws_romindly`. Source packages override the apt packages of the same name (the workspace overlay takes precedence). This tutorial assumes the apt route.

### Step 3: Set TURTLEBOT3_MODEL

TB3's launch files use the `TURTLEBOT3_MODEL` environment variable to decide which robot's URDF to load — **if it is not set, they exit with an error**. This tutorial series consistently uses `burger`:

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
echo $TURTLEBOT3_MODEL   # should print burger
```

Differences between the three models:

| Model | Sensors | Max speed (linear/angular) | Characteristics |
| --- | --- | --- | --- |
| `burger` | 360° LiDAR (LDS) | 0.22 m/s / 2.84 rad/s | Smallest and lightest; first choice for teaching 2D mapping and navigation |
| `waffle` | LiDAR + RealSense depth camera | 0.26 m/s / 1.82 rad/s | Wider base, with vision |
| `waffle_pi` | LiDAR + Raspberry Pi camera | 0.26 m/s / 1.82 rad/s | Same base as waffle, different camera |

`burger` is sufficient for the experiments from this chapter through chapter 40 (mapping and navigation only need the LiDAR); experiments involving a camera will explicitly note when to switch to `waffle_pi`.

### Step 4: Launch the simulation world for the first time

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Expected: the Gazebo window opens showing an arena with a hexagonal fence and a few cylinders inside, with a small black robot (burger) in the center. In the terminal you should see:

```
[ INFO] [...]: Finished loading Gazebo ROS API Plugin.
[ INFO] [...]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [...]: Physics dynamic reconfigure ready.
```

> **The first launch is slow (possibly 1–3 minutes of black screen)**: on its first run, Gazebo tries to contact the online model database to download basic models such as `sun` and `ground_plane` and build a local cache (`~/.gazebo/models/`). Just be patient — the second launch is fast. If a blocked network causes it to hang indefinitely, see Common Issues below.

Confirm the ROS-side interfaces are working (in another terminal):

```bash
rostopic list
```

You should see topics such as `/scan`, `/odom`, `/cmd_vel`, `/imu`, and `/clock` — the next part examines each of them.

To shut down the simulation: press `Ctrl+C` in the roslaunch terminal and let it exit cleanly on its own (Gazebo shuts down slowly — don't hammer the keys).

## Explanation

- `turtlebot3_world.launch` does three things: starts Gazebo and loads the `turtlebot3_world` world file; reads the URDF according to `TURTLEBOT3_MODEL` and spawns the robot into the simulation; and starts `robot_state_publisher` to publish the robot's internal TF (covered in detail in part 03).
- Gazebo is actually two processes: `gzserver` (physics computation, headless) + `gzclient` (the 3D display window). After an abnormal exit, gzserver often lingers in the background, causing errors on the next launch — this is the single most frequent problem with TB3 simulation; see below.
- Other worlds to try: `turtlebot3_empty_world.launch` (empty ground) and `turtlebot3_house.launch` (a multi-room house with many models — slower on first load, well suited to the chapter 40 navigation experiments).

## Exercises

1. Close the current simulation, launch `turtlebot3_house.launch`, and browse the whole house in Gazebo with the mouse (left button to pan, scroll wheel to zoom, Shift+left button to rotate the view).
2. Temporarily switch the model to waffle: `TURTLEBOT3_MODEL=waffle roslaunch turtlebot3_gazebo turtlebot3_world.launch`, and compare the robot's appearance (this prefix syntax applies only to this one command and does not affect the setting in bashrc).
3. Deliberately launch once without the environment variable: `env -u TURTLEBOT3_MODEL roslaunch turtlebot3_gazebo turtlebot3_world.launch`, observe the error message, and remember what it looks like.

## Common Issues

**Q1: Launch fails with `[Err] [RTShaderSystem.cc] ... unable to find ...`, or `gzserver` crashes outright with an address-in-use error.**
The previous simulation did not exit cleanly and gzserver is lingering. Clean up and restart:

```bash
killall -9 gzserver gzclient
```

Make it a habit: after every `Ctrl+C`, wait until the terminal fully returns to the prompt before launching again.

**Q2: In a virtual machine, Gazebo shows a black screen / corrupted graphics / crashes instantly (errors typically mention `OGRE EXCEPTION` or `VBO`).**
The VM's 3D acceleration conflicts with Gazebo's OpenGL requirements. Two fixes:

```bash
# Option 1: force software rendering (slow but stable)
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Option 2: disable "3D graphics acceleration" in the VM settings. The edge computing unit used by this kit is a physical machine with integrated graphics, so this rarely occurs there; the issue is mostly seen on students' own VMware/VirtualBox environments.

**Q3: Error `TURTLEBOT3_MODEL is not set`.**
The environment variable did not take effect. Newly opened terminals need a fresh `source ~/.bashrc` (or confirm the echo of step 3 actually got written: `grep TURTLEBOT3 ~/.bashrc`).

**Q4: The world loads but the arena/furniture is missing, and the terminal keeps printing `waiting for model` or model download timeouts.**
When Gazebo cannot find a model locally, it tries to download it from the online model database and waits forever if the network is blocked. The models that ship with TB3 are actually all in the local package; confirm the model path includes them:

```bash
echo $GAZEBO_MODEL_PATH
```

It should normally contain `/opt/ros/noetic/share/turtlebot3_gazebo/models` (appended automatically by the package environment hooks when you `source /opt/ros/noetic/setup.bash`). If it is empty, add it manually:

```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

**Q5: The simulation is very laggy, and the Real Time Factor in Gazebo's bottom status bar is far below 1.0.**
Insufficient machine performance or software rendering. For teaching purposes, RTF ≥ 0.8 is acceptable; turn off unnecessary Gazebo window effects (left panel → shadows), or switch to a lightweight world such as `turtlebot3_empty_world.launch`.

Next part: [02 Keyboard Teleop and Topic Observation](02_teleop_and_topics.md)
