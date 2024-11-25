# 使用官方ROS2 Iron基础镜像
FROM ros:iron

# 避免安装过程中的交互
ENV DEBIAN_FRONTEND=noninteractive

# 安装常用开发工具和项目依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-iron-rqt* \
    ros-iron-rviz2 \
    ros-iron-tf2-ros \
    ros-iron-tf2-tools \
    ros-iron-xacro \
    ros-iron-nav2-bringup \
    ros-iron-navigation2 \
    ros-iron-slam-toolbox \
    ros-iron-robot-localization \
    git \
    vim \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# 安装Python依赖
RUN pip3 install \
    numpy \
    matplotlib \
    opencv-python \
    torch \
    torchvision \
    ultralytics

# 设置环境变量
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# 设置入口点
ENTRYPOINT ["/bin/bash"]