# 1. 选择 CUDA 11.8 + Ubuntu 22.04 作为基础镜像，在其上安装 ROS2 Humble
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# 2. 基础工具与 locale
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 3. 配置 ROS2 Humble 软件源并安装 ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-opencv \
    geographiclib-tools \
    libgeographic-dev \
    libserial-dev \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# 4. 安装 Python 依赖（不包含 torch/torchvision/torchaudio）
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --upgrade pip && \
    pip3 install --ignore-installed sympy -r /tmp/requirements.txt

# 5. 安装与 CUDA 11.8 匹配的 PyTorch（GPU 版）
RUN pip3 install --index-url https://download.pytorch.org/whl/cu118 \
    torch torchvision torchaudio

# 6. 拷贝源码
COPY . /workspace
WORKDIR /workspace

# 7. 构建 ROS2 工作区, 先清理编译残留，再编译, 新增 --packages-skip 跳过 marnav_vis 包编译,最后由自己编译
RUN rm -rf build/ install/ log/ && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-skip marnav_vis

# 8. 设置环境变量
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONUNBUFFERED=1

# 9. 启动入口（使用 GPU 时，运行容器需加 --gpus all）
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec bash"]