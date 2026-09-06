# Romindly ROS1 Noetic 基础镜像
# 用于 x86 边缘单元部署与 CI，锁定 Noetic 依赖（Noetic 已 EOL，镜像即快照）
#
# 构建: docker build -t romindly/ros1-noetic .
# 运行: docker run -it --net=host romindly/ros1-noetic

FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]

# ROS apt key 已于 2025 年轮换，老镜像内置的 key 会导致 apt update 失败 (EXPKEYSIG)
# 用 ADD 直接拉取最新 key，覆盖镜像内两处可能的 keyring 位置
ADD https://raw.githubusercontent.com/ros/rosdistro/master/ros.key /usr/share/keyrings/ros1-latest-archive-keyring.gpg
RUN chmod 644 /usr/share/keyrings/ros1-latest-archive-keyring.gpg && \
    cp /usr/share/keyrings/ros1-latest-archive-keyring.gpg /etc/apt/trusted.gpg.d/ros-latest.gpg

# 换用清华镜像源加速国内下载（海外部署可注释掉）
RUN sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g; s|http://ports.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# 常用工具、教程涉及的 ROS 包、以及第二/三阶段的系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim wget curl ca-certificates software-properties-common \
    python3-catkin-tools python3-vcstool python3-rosdep \
    build-essential cmake \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    ros-noetic-tf2-tools \
    ros-noetic-rqt-graph \
    ros-noetic-actionlib \
    libsdl1.2-dev libsdl-image1.2-dev libnetpbm10-dev qt5-image-formats-plugins \
    ros-noetic-libg2o ros-noetic-mbf-costmap-core ros-noetic-mbf-msgs \
    ros-noetic-sparse-bundle-adjustment \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# GTSAM (LIO-SAM 依赖, borglab PPA)
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
    apt-get update && apt-get install -y --no-install-recommends \
    libgtsam-dev libgtsam-unstable-dev && rm -rf /var/lib/apt/lists/*

# Livox-SDK (livox_ros_driver 依赖, 源码安装)
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git /tmp/Livox-SDK && \
    cmake -S /tmp/Livox-SDK -B /tmp/Livox-SDK/build -DCMAKE_BUILD_TYPE=Release && \
    make -C /tmp/Livox-SDK/build -j"$(nproc)" && make -C /tmp/Livox-SDK/build install && \
    rm -rf /tmp/Livox-SDK

# 创建工作空间并拉取代码 (含子模块)
WORKDIR /root/ws_romindly
COPY ros1.repos /root/ws_romindly/
RUN mkdir -p src && vcs import --recursive src < ros1.repos

# 安装 rosdep 依赖并编译
RUN source /opt/ros/noetic/setup.bash && \
    apt-get update && \
    (rosdep init 2>/dev/null; rosdep update) && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -j"$(nproc)" && \
    rm -rf /var/lib/apt/lists/*

# 进容器自动 source 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ws_romindly/devel/setup.bash ] && source /root/ws_romindly/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
