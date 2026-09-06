# Docker 部署

ROS Noetic 已于 2025-05 停止维护（EOL），apt 源不再更新。用 Docker 镜像把整套依赖做成快照，是边缘单元批量部署与长期维护的推荐方式。

## 构建镜像

```bash
cd ~/ws_romindly/src/romindly_ros1_workspace/docker
cp ../ros1.repos .        # Dockerfile 需要 repos 清单
docker build -t romindly/ros1-noetic .
```

## 运行

```bash
# --net=host: 容器与宿主机共享网络，ROS 节点可直接互通
# --privileged -v /dev:/dev: 需要访问激光雷达等 USB 设备时加上
docker run -it --net=host --privileged -v /dev:/dev romindly/ros1-noetic
```

## 图形界面 (RViz / rqt)

```bash
xhost +local:docker
docker run -it --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  romindly/ros1-noetic
# 容器内: roslaunch urdf_demo display.launch
```

## 挂载本地源码开发

```bash
docker run -it --net=host \
  -v ~/ws_romindly/src:/root/ws_romindly/src \
  romindly/ros1-noetic
# 容器内: cd /root/ws_romindly && catkin_make
```

## 常见问题

- **容器内 rosdep 慢/失败**：国内网络建议配置代理，或使用 `rosdepc`（小鱼版国内源）。
- **设备权限**：宿主机上先给用户加 dialout 组：`sudo usermod -aG dialout $USER`。
- **开机自启容器**：`docker run` 加 `--restart=always -d`，或用 systemd 管理，见 [50 部署运维](../docs/50_部署运维/)。
