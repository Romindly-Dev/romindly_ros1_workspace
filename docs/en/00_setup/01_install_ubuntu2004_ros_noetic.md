# Installing ROS Noetic on Ubuntu 20.04

> 中文版 / Chinese: [01_Ubuntu20.04_ROS_Noetic安装.md](../../00_环境搭建/01_Ubuntu20.04_ROS_Noetic安装.md)

## Goal

Complete a full installation of ROS Noetic on an x86_64 edge computing unit (Ubuntu 20.04 Focal), covering apt source configuration, the desktop-full installation, rosdep initialization (including an alternative for restricted networks in China), environment variable setup, and finally verifying the installation with turtlesim.

> **Note**: ROS Noetic officially reached EOL (end of maintenance) in May 2025. The apt sources are still available and packages install normally, but there are no further security updates or bug fixes. For production environments it is recommended to freeze the runtime environment with Docker; see the [Docker deployment guide](../../../docker/README.md).

## Background

ROS Noetic is the last LTS release of ROS1 and only supports Ubuntu 20.04. Installation takes four steps: add the package source and key → install via apt → initialize rosdep (the dependency resolution tool) → configure environment variables. Since access to the official sources is slow from networks in China, this guide also provides the Tsinghua mirror and the rosdepc alternative.

## Step-by-Step Instructions

### 1. Confirm the system version

```bash
lsb_release -a
```

Expected output:

```
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.6 LTS
Release:        20.04
Codename:       focal
```

If your system is not 20.04 (focal), Noetic cannot be installed via apt — resolve the OS version issue first.

### 2. Add the ROS apt source

For users in China, the Tsinghua mirror is recommended (much faster):

```bash
sudo sh -c '. /etc/os-release && echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $VERSION_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```

To use the official source instead, replace the URL with `http://packages.ros.org/ros/ubuntu/`.

### 3. Add the key

```bash
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Expected output:

```
OK
```

If `raw.githubusercontent.com` is unreachable, use a keyserver instead:

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 4. Install ros-noetic-desktop-full

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
```

desktop-full includes the ROS core, the rqt tool suite, RViz, Gazebo 11, and common perception libraries. It requires roughly 500MB+ of downloads and takes about 3GB of disk space once installed.

Verify the package is installed:

```bash
apt list --installed 2>/dev/null | grep ros-noetic-desktop-full
```

Expected output:

```
ros-noetic-desktop-full/focal,now 1.5.0-1focal.xxxxxxxx amd64 [installed]
```

### 5. Initialize rosdep

rosdep resolves and installs the system dependencies of ROS packages. The official initialization procedure:

```bash
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
```

**On networks in China, `rosdep init` and `rosdep update` are very likely to time out.** The recommended alternative is rosdepc (the "c" stands for China), maintained by the local community:

```bash
sudo apt install -y python3-pip
pip3 install rosdepc
sudo rosdepc init
rosdepc update
```

Expected output (at the end):

```
updated cache in /home/<username>/.ros/rosdep/sources.cache
```

From this point on, wherever a `rosdep` command appears in the documentation, you can substitute `rosdepc` equivalently (the arguments are identical).

### 6. Configure environment variables

Write the ROS environment into `.bashrc` so it takes effect automatically in every new terminal:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
echo $ROS_DISTRO
```

Expected output:

```
noetic
```

### 7. Verify the installation: roscore + turtlesim

Open the **first terminal** and start the ROS Master:

```bash
roscore
```

Expected output (at the end):

```
started core service [/rosout]
```

Open a **second terminal** and start the turtle simulation:

```bash
rosrun turtlesim turtlesim_node
```

A window with a blue background should pop up, with a turtle in the middle.

Open a **third terminal** and start keyboard control:

```bash
rosrun turtlesim turtle_teleop_key
```

Keep this terminal focused and press the arrow keys — the turtle should move accordingly. The installation is now verified. When done, press `Ctrl+C` in each terminal to exit.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `apt update` reports GPG error `NO_PUBKEY F42ED6FBAB17C654` | Key was not added successfully | Redo step 3 using the keyserver method |
| `apt update` hangs or is extremely slow | Using the official source on a restricted network | Switch to the Tsinghua mirror (step 2); the system sources can also be switched to `mirrors.tuna.tsinghua.edu.cn` |
| `sudo rosdep init` reports `Website may be down` | Cannot reach raw.githubusercontent.com | Use rosdepc instead (step 5) |
| `rosdep update` times out | Same as above | Use `rosdepc update` instead |
| `roscore: command not found` | Environment not sourced | Run `source /opt/ros/noetic/setup.bash` and confirm it was written to `.bashrc` |
| turtlesim window does not appear (over SSH) | No graphical environment | Run on the local desktop, or in a session with X forwarding (`ssh -X`) |
| `Unable to register with master node` | roscore is not running | Start `roscore` in a separate terminal first |
