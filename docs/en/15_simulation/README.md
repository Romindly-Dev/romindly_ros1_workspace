# 15 Simulation Basics

> 中文版 / Chinese: [README.md](../../15_仿真入门/README.md)

Why learn simulation first: even without a physical robot, you can walk through the entire **mapping → localization → navigation** pipeline in Gazebo. All experiments in chapters 20/30/40 run on this TurtleBot3 simulation environment; once mastered, everything transfers as-is to the real robot.

## Contents

| Part | Content | What you will learn |
| --- | --- | --- |
| [01 Installing TurtleBot3 and Gazebo](01_turtlebot3_gazebo_install.md) | Verifying Gazebo, installing the TB3 trio (apt recommended), the `TURTLEBOT3_MODEL` environment variable, launching the simulation world for the first time | Get `turtlebot3_world.launch` running |
| [02 Keyboard Teleop and Topic Observation](02_teleop_and_topics.md) | Keyboard teleoperation; observing `/scan` `/odom` `/cmd_vel` `/imu` with rostopic/rqt_graph/RViz | Understand the data flow of the simulated robot |
| [03 The TF Tree and Sensors in Simulation](03_tf_and_sensors_in_sim.md) | Exporting the TF tree with view_frames and walking through it layer by layer; the division of labor between robot_state_publisher and the Gazebo plugins; `/clock` and `use_sim_time` | Understand where the `/scan` + TF required by mapping algorithms come from |

## Prerequisites

- Complete [00 Environment Setup](../00_setup/01_install_ubuntu2004_ros_noetic.md) (desktop-full already includes Gazebo 11)
- Finish [10 ROS1 Basics](../10_ros1_basics/README.md), especially [02 Topic Communication](../10_ros1_basics/02_topics.md), [06 TF2 Coordinate Transforms](../10_ros1_basics/06_tf2_transforms.md), [07 URDF Robot Modeling](../10_ros1_basics/07_urdf_modeling.md), and [08 rosbag and Debugging Tools](../10_ros1_basics/08_rosbag_and_debugging.md) — this chapter refers back to them repeatedly

After finishing this chapter you are ready for [20 2D SLAM](../20_2d_slam/README.md).
