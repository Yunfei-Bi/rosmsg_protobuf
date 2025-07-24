FROM ubuntu:20.04
ENV DEBIAN FRONTEND=noninteractive
RUN apt-get clean && \
    apt-get autoclean
RUN apt update && \
    apt install -y \
    curl \
    lsb-release \
    gnupg gnupg1 gnupg2 \
    gdb

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt update && \
    apt install -y ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bahsrc
RUN apt update && \
    apt install -y \
    vim \
    htop \
    apt-utils \
    curl \
    cmake \
    net-tools

COPY install/abseil /tmp/install/abseil
RUN /tmp/install/abseil/install_abseil.sh

RUN echo "source /work/devel/setup.bash" >> ~/.bashrc

WORKDIR /work
# WORKDIR /work
# 作用：设置容器的工作目录（相当于 cd /work）。
# 特点：
# 后续的 RUN、CMD、COPY、ADD 等命令都会在该目录下执行。
# 如果目录不存在，会自动创建。