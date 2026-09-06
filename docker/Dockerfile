# Romindly ROS1 Noetic 基础镜像
# 用于 x86 边缘单元部署与 CI，锁定 Noetic 依赖（Noetic 已 EOL，镜像即快照）
#
# 构建: docker build -t romindly/ros1-noetic .
# 运行: docker run -it --net=host romindly/ros1-noetic

FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]

# 换用清华镜像源加速国内下载（海外部署可注释掉）
RUN sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g; s|http://ports.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# 常用工具与本系列教程涉及的 ROS 包
RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim wget curl \
    python3-catkin-tools python3-vcstool python3-rosdep \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    ros-noetic-tf2-tools \
    ros-noetic-rqt-graph \
    ros-noetic-actionlib \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间并拉取代码
WORKDIR /root/ws_romindly
COPY ros1.repos /root/ws_romindly/
RUN mkdir -p src && vcs import src < ros1.repos || true

# 编译（CI 中重新挂载源码时可跳过）
RUN source /opt/ros/noetic/setup.bash && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    catkin_make || true && \
    rm -rf /var/lib/apt/lists/*

# 进容器自动 source 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ws_romindly/devel/setup.bash ] && source /root/ws_romindly/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
