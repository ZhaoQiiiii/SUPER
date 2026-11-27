# Dockerfile — SUPER 独立运行环境（Ubuntu 20.04 + ROS Noetic + MAVROS）。
#
# 只包含规划链路（super_planner / rog_map / mission_planner / quadrotor_msgs）
# 的构建与运行依赖，不依赖其他镜像；代码通过挂载使用。
#
# 未安装仿真/可视化包（rviz_plugins、marsim_render、perfect_drone_sim）所需的
# Qt/GL/GLFW/GLEW/OpenCV，构建时需跳过它们：
#   cd /workspace/SUPER && catkin_make -DBUILD_TYPE=Release \
#     -DCATKIN_BLACKLIST_PACKAGES="rviz_plugins;marsim_render;perfect_drone_sim"
#
# 构建（上下文是本 docker/ 目录）：
#   docker build -t super:noetic runtime/Robots/quadrotors/src/SUPER/docker
#
# 运行示例：
#   docker run --rm -it --network host \
#     -v <path-to-SUPER>:/workspace/SUPER super:noetic
FROM ubuntu:20.04

LABEL maintainer="iinnoovo123@gmail.com"
ENV DEBIAN_FRONTEND=noninteractive

ARG HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy

# =====================================
# 1. 网络代理与镜像源替换
# =====================================
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
  sed -i 's/security.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list && \
  sed -i 's/^deb /deb [trusted=yes] /' /etc/apt/sources.list

# =======================================================
# 2. 系统基础环境初始化
# =======================================================
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  tzdata \
  ca-certificates \
  gnupg2 \
  lsb-release \
  software-properties-common && \
  rm -rf /var/lib/apt/lists/*

# =======================================================
# 3. 安装 ROS Noetic（ros-base 已包含 roscpp/rospy/消息包）
# =======================================================
ENV ROS_DISTRO=noetic
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list && \
  apt-get update && \
  apt-get install -y --no-install-recommends \
  ros-noetic-ros-base && \
  rm -rf /var/lib/apt/lists/*

# =======================================================
# 4. SUPER 规划链路依赖与 MAVROS
# =======================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  wget \
  libeigen3-dev \
  libpcl-dev \
  libyaml-cpp-dev \
  libdw-dev \
  libncurses5-dev \
  libncursesw5-dev \
  libboost-system-dev \
  libboost-filesystem-dev \
  libboost-regex-dev \
  libboost-iostreams-dev \
  ros-noetic-tf \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-eigen-conversions \
  ros-noetic-mavros \
  ros-noetic-mavros-extras && \
  wget -T 30 -t 3 https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
  bash install_geographiclib_datasets.sh && \
  rm install_geographiclib_datasets.sh && \
  rm -rf /var/lib/apt/lists/*

# =======================================================
# 5. 运行环境
# =======================================================
# README 要求的 Eigen 软链；进入容器自动加载 ROS 环境。
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen && \
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

ENV DISABLE_ROS1_EOL_WARNINGS=1

WORKDIR /workspace
CMD ["/bin/bash"]
