# 03 Docker 部署

> English version: [03_docker_deployment.md](../en/50_deployment/03_docker_deployment.md)

## 目标

在 [docker/README.md](https://github.com/Romindly-Dev/romindly_ros1_workspace/blob/main/docker/README.md) 的速查基础上，完整走通：构建镜像 → 带设备/网络/显示运行 → 离线分发到产线 → 容器开机自启 → 数据持久化 → 多容器编排。

## 背景：EOL 之后，Docker 是正解

ROS Noetic 已于 2025-05 EOL，这带来两个现实问题：apt 源不再有更新（今天能装的版本组合，明天官方源调整后未必还能装出一模一样的）；而 Ubuntu 20.04 本身也在走向标准支持结束。裸机环境是"活的"，装一台是一台，无法保证第 10 台和第 1 台一致。

Docker 镜像把**整套依赖做成只读快照**：ROS、apt 包、编译产物全部冻结在一个镜像文件里，`docker build` 一次，产线 N 台机器 `docker load` 出来的环境逐字节相同；升级 = 换镜像 tag，回滚 = 换回旧 tag。这正是 EOL 系统长期维护的正确姿势（升级策略详见 [04 节](04_性能优化与运维.md)）。

## Dockerfile 逐段讲解

完整文件见 [docker/Dockerfile](https://github.com/Romindly-Dev/romindly_ros1_workspace/blob/main/docker/Dockerfile)：

```dockerfile
FROM ros:noetic-ros-base-focal
```

基于 OSRF 官方镜像：Ubuntu 20.04 (focal) + ros-base（无 GUI），ROS apt 源和 key 已配好——**镜像内不会遇到 EXPKEYSIG 问题**，因为镜像里的 key 随基础镜像更新。

```dockerfile
RUN sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g; ...' /etc/apt/sources.list
```

换清华镜像加速国内构建；海外部署注释掉即可。

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim wget curl python3-catkin-tools python3-vcstool python3-rosdep \
    ros-noetic-xacro ros-noetic-robot-state-publisher ... \
    && rm -rf /var/lib/apt/lists/*
```

一层装齐工具与教程用到的 ROS 包。`--no-install-recommends` 减体积；结尾删 apt 缓存是 Docker 惯例（每个 RUN 是一层，缓存留着白占镜像体积）。

```dockerfile
WORKDIR /root/ws_romindly
COPY ros1.repos /root/ws_romindly/
RUN mkdir -p src && vcs import src < ros1.repos || true
RUN source /opt/ros/noetic/setup.bash && \
    apt-get update && rosdep install --from-paths src --ignore-src -r -y || true && \
    catkin_make || true && rm -rf /var/lib/apt/lists/*
```

构建时把 `ros1.repos` 清单里的仓库拉进镜像并编译——**代码和编译产物也进快照**。`|| true` 让个别包失败不中断构建（教学镜像的宽松策略；生产镜像建议去掉，让失败暴露出来）。

```dockerfile
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && ...
CMD ["bash"]
```

交互进容器时自动 source；默认命令是 bash（生产自启时会覆盖 CMD，见下文）。

## 构建与运行

```bash
cd ~/ws_romindly/src/romindly_ros1_workspace/docker
cp ../ros1.repos .
docker build -t romindly/ros1-noetic:1.0 .     # 用版本号 tag，别只用 latest
```

预期结尾输出 `Successfully tagged romindly/ros1-noetic:1.0`。

三种典型运行姿势（可叠加）：

```bash
# 1. 网络：--net=host 容器与宿主机共享网络栈，ROS1 节点端口随机、必须 host 模式
docker run -it --net=host romindly/ros1-noetic:1.0

# 2. 设备透传：访问雷达/底盘串口
docker run -it --net=host --privileged -v /dev:/dev romindly/ros1-noetic:1.0
# 容器内验证: ls -l /dev/rplidar

# 3. 图形（rviz）：透传 X11
xhost +local:docker
docker run -it --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix romindly/ros1-noetic:1.0
```

> `--privileged -v /dev:/dev` 简单粗暴但权限全开；收紧版是逐个 `--device=/dev/rplidar`，代价是热插拔后需重启容器。udev 规则在**宿主机**上配（[45 章](../45_传感器驱动/04_udev规则与设备管理.md)），容器里看到的软链是宿主机生成的。

## 镜像分发：离线导入产线

产线机器往往无外网，用 save/load 走 U 盘或内网：

```bash
# 开发机导出（gzip 压缩，Noetic 基础镜像 gz 后约 1–2 GB）
docker save romindly/ros1-noetic:1.0 | gzip > romindly-ros1-1.0.tar.gz

# 产线机导入
gunzip -c romindly-ros1-1.0.tar.gz | docker load
docker images    # 应看到 romindly/ros1-noetic  1.0
```

配一个 `sha256sum` 校验文件随包发放，导入前校验完整性。

## 容器开机自启：两种方式

| 方式 | 做法 | 优点 | 缺点 |
| --- | --- | --- | --- |
| Docker 自带 restart 策略 | `docker run -d --restart=always ...` | 一条命令搞定，docker daemon 起来就拉容器 | 依赖表达不了（等设备/网络）、停启粒度粗、与其他 systemd 服务无法编排 |
| systemd 管理容器 | 写 service，ExecStart 为 `docker run` | 复用 [02 节](02_systemd开机自启.md)全部能力：After/Requires、KillSignal、journalctl | 多一个文件要维护 |

方式一：

```bash
docker run -d --name ros-robot --restart=always --net=host \
  --privileged -v /dev:/dev romindly/ros1-noetic:1.0 \
  bash -c "source /root/ws_romindly/devel/setup.bash && roslaunch robot_bringup robot.launch"
```

方式二（推荐，与 02 节体系统一），`/etc/systemd/system/ros-robot-docker.service`：

```ini
[Unit]
Description=ROS1 Robot (Docker)
After=docker.service network-online.target dev-rplidar.device
Requires=docker.service dev-rplidar.device

[Service]
ExecStartPre=-/usr/bin/docker rm -f ros-robot
ExecStart=/usr/bin/docker run --name ros-robot --net=host --privileged \
  -v /dev:/dev -v /home/iot/robot_data:/root/robot_data \
  romindly/ros1-noetic:1.0 \
  bash -c "source /root/ws_romindly/devel/setup.bash && exec roslaunch robot_bringup robot.launch"
ExecStop=/usr/bin/docker stop -t 20 ros-robot
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

注意：两种方式**二选一**——用 systemd 就不要再加 `--restart=always`，否则两个管理者会打架。

## 数据持久化

容器文件系统是易失的（`docker rm` 即消失），地图、日志、rosbag 必须落宿主机 volume：

```bash
mkdir -p ~/robot_data/{maps,logs,bags}
docker run ... \
  -v ~/robot_data/maps:/root/robot_data/maps \
  -v ~/robot_data/logs:/root/.ros/log \
  -v ~/robot_data/bags:/root/robot_data/bags \
  romindly/ros1-noetic:1.0
```

这样 `map_server` 存图、rosout 日志、黑匣子 rosbag（[04 节](04_性能优化与运维.md)）都持久保存，且换镜像版本时数据不动。

## 多容器编排（一句话版）

系统大了以后可用 docker compose 把 roscore 和应用拆成两个容器（对应 02 节的双服务思想）：

```yaml
services:
  roscore:
    image: romindly/ros1-noetic:1.0
    network_mode: host
    command: roscore
    restart: unless-stopped
  robot:
    image: romindly/ros1-noetic:1.0
    network_mode: host
    privileged: true
    volumes: ["/dev:/dev", "~/robot_data/maps:/root/robot_data/maps"]
    depends_on: [roscore]
    command: bash -c "source /root/ws_romindly/devel/setup.bash && exec roslaunch robot_bringup robot.launch --wait"
    restart: unless-stopped
```

`docker compose up -d` 一键拉起，`--wait` 让应用容器等 roscore 就绪。

## 常见问题

- **容器内 `rostopic list` 连不上宿主机 master**：忘了 `--net=host`。ROS1 端口随机分配，端口映射（-p）方案不可行，host 网络是唯一省心解。
- **rviz 报 could not connect to display**：宿主机没执行 `xhost +local:docker`，或漏挂 `/tmp/.X11-unix`。ssh 远程场景见 [04 节](04_性能优化与运维.md)远程运维段。
- **容器内看不到 /dev/rplidar**：软链是宿主机 udev 生成的，先在宿主机确认 45 章规则生效；再确认用了 `-v /dev:/dev --privileged`。
- **build 时 vcs import 拉取慢/失败**：编辑 `ros1.repos` 换镜像前缀，或在有外网的开发机构建后 save/load 分发——产线机器永远不需要构建。
- **改了代码要重打镜像太慢**：开发期用"挂载源码"模式（`-v ~/ws_romindly/src:/root/ws_romindly/src` 容器内重编），发布时才重新 build 镜像并升 tag。
