# Creating the Workspace and Pulling the Code

> 中文版 / Chinese: [02_工作空间与代码拉取.md](../../00_环境搭建/02_工作空间与代码拉取.md)

## Goal

Create the catkin workspace `~/ws_romindly`, pull all repositories of this learning kit in bulk with vcstool, install dependencies, complete the first build, and finally verify the workspace works using a URDF demo.

## Background

catkin is the build system of ROS1. A catkin workspace contains three directories:

| Directory | Purpose |
| --- | --- |
| `src/` | Source directory; all packages live here — the only directory you manage by hand |
| `build/` | Intermediate build artifacts (CMake cache, object files), generated automatically by `catkin_make` |
| `devel/` | Build results (executables, setup.bash, etc.); source it to use the packages in the workspace |

This kit consists of multiple Git repositories, described by a single `ros1.repos` manifest file. `vcstool` reads the manifest and pulls all repositories with one command.

## Step-by-Step Instructions

### 1. Create the workspace directory

```bash
mkdir -p ~/ws_romindly/src
cd ~/ws_romindly
```

### 2. Install vcstool and git

```bash
sudo apt update
sudo apt install -y git python3-vcstool
```

Verify:

```bash
vcs --version
```

Expected output (the version number may differ):

```
vcs 0.2.x
```

### 3. Clone the main repository

```bash
cd ~/ws_romindly
git clone https://github.com/Romindly-Dev/romindly_ros1_workspace.git src/romindly_ros1_workspace
```

> If GitHub access is slow in your region, you can temporarily use a mirror accelerator (e.g. the `https://ghproxy.com/` prefix) or configure a proxy.

### 4. Pull all repositories in bulk with vcs

The `ros1.repos` file inside the main repository lists all dependency repositories of the kit:

```bash
cd ~/ws_romindly
vcs import src < src/romindly_ros1_workspace/ros1.repos
```

Expected output (one line per repository; `.` means success):

```
=== src/xxx (git) ===
Cloned https://github.com/... to src/xxx
...
```

After pulling, confirm:

```bash
ls ~/ws_romindly/src
```

You should see `romindly_ros1_workspace` plus the package directories listed in the manifest.

### 5. Install dependencies with rosdep

```bash
cd ~/ws_romindly
rosdep install --from-paths src --ignore-src -r -y
```

> On networks in China, use rosdepc (usage is identical):
>
> ```bash
> rosdepc install --from-paths src --ignore-src -r -y
> ```

Expected output (at the end):

```
#All required rosdeps installed successfully
```

### 6. Build

```bash
cd ~/ws_romindly
catkin_make
```

The first build takes several minutes. Expected output (at the end):

```
[100%] Built target ...
```

After the build completes, two new directories, `build/` and `devel/`, appear in the workspace.

### 7. Source the workspace environment

```bash
source ~/ws_romindly/devel/setup.bash
```

It is recommended to write this into `.bashrc` so you don't have to run it manually every time:

```bash
echo "source ~/ws_romindly/devel/setup.bash" >> ~/.bashrc
```

> `devel/setup.bash` automatically chain-loads `/opt/ros/noetic/setup.bash`; just make sure the workspace line comes after the ROS line in `.bashrc`.

Verify that packages can be found:

```bash
rospack find urdf_demo
```

Expected output:

```
/home/<username>/ws_romindly/src/.../urdf_demo
```

### 8. Verify: launch the URDF demo

```bash
roslaunch urdf_demo display.launch
```

Expected result: an RViz window opens showing a blue differential-drive cart model; in the left panel, the RobotModel status is OK. A joint_state_publisher slider window also appears — dragging the sliders makes the wheels rotate.

Press `Ctrl+C` to exit. The workspace setup is now complete.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `git clone` times out / speed stuck at KB level | Restricted GitHub access | Use a mirror prefix or a proxy; the same applies to sub-repositories pulled by `vcs import` — you can edit the URLs in `ros1.repos` |
| `vcs: command not found` | vcstool not installed | `sudo apt install python3-vcstool` (note the package name is not vcstool) |
| `rosdep install` reports `Cannot locate rosdep definition` for some packages | rosdep cache not updated, or no local mirror configured | Run `rosdepc update` and retry; the `-r` flag already allows skipping failures and continuing |
| `catkin_make` cannot find a package (`Could not find a package configuration file`) | Dependencies not fully installed | Rerun step 5; if something is still missing, install it manually by the reported name: `sudo apt install ros-noetic-<package-name (underscores replaced with hyphens)>` |
| `catkin_make` reports CMake version / mixed-tool errors | The workspace was previously built with catkin build | Delete `build/` and `devel/` and rerun `catkin_make` (the two tools cannot be mixed in the same workspace) |
| `roslaunch` cannot find urdf_demo | devel/setup.bash not sourced | Run step 7, or open a new terminal (if it is already in .bashrc) |
| Model appears all white in RViz / `No transform` errors | Wrong Fixed Frame | Change the Fixed Frame at the top left of RViz to `base_link` (or the root frame specified in the launch file) |
