FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04 AS base

# Never interact with user
ENV DEBIAN_FRONTEND=noninteractive

# Upgrade packages
RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install language
RUN apt-get update && apt-get install -y \
        locales \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install timezone
RUN apt-get update && apt-get install -y \
        tzdata \
    && rm -rf /var/lib/apt/lists/* \
    && ln -fs /usr/share/zoneinfo/Europe/Warsaw /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata

# Create non-root user and add sudo support
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN apt-get update && apt-get install -y \
        sudo \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

FROM base AS ros2

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
        curl \
        gnupg2 \
        software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Env vars for ROS2
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

FROM ros2 AS dev

# Install dev packages
RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    clang \
    cmake \
    g++ \
    gcc \
    gdb \
    git \
    lsb-release \
    make \
    nano \
    pkg-config \
    python3 \
    python3-pip \
    tar \
    unzip \
    vim \
    wget \
&& rm -rf /var/lib/apt/lists/*

# Install ROS dev packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-humble-ament-* \
        ros-humble-rviz2 \
        ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# Initialize ROS dependencies
RUN rosdep init || echo ""

# Install cone_detection dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        libarmadillo-dev \        
        libopencv-dev \
        ros-humble-cv-bridge \
        ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# ONNX Runtime
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-x64-gpu-cuda12-1.18.1.tgz \
    && wget https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-x64-1.18.1.tgz \
    && tar -xvzf onnxruntime-linux-x64-gpu-cuda12-1.18.1.tgz \
    && tar -xvzf onnxruntime-linux-x64-1.18.1.tgz \
    && mkdir -p /usr/local/onnxruntime-libs \
    && mv onnxruntime-linux-x64-gpu-1.18.1 onnxruntime-linux-x64-1.18.1 /usr/local/onnxruntime-libs/ \
    && rm onnxruntime-linux-x64-gpu-cuda12-1.18.1.tgz onnxruntime-linux-x64-1.18.1.tgz

# Install path_planner dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        liblemon-dev \
    && rm -rf /var/lib/apt/lists/*

FROM dev AS gpu

# Expose Nvidia driver to allow OpenGL
# Dependencies for GLVND, X11, Vulkan
RUN apt-get update && apt-get install -y --no-install-recommends \
        libegl1 \
        libgl1 \
        libglvnd0 \
        libglx0 \
        libx11-6 \
        libxext6 \
    && rm -rf /var/lib/apt/lists/*
# Env vars for NVIDIA Container Toolkit
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV QT_X11_NO_MITSHM=1

# Vulkan support
RUN apt-get update && apt-get install -y --no-install-recommends \ 
        libglu1-mesa-dev \
        libvulkan1 \
        mesa-utils \
        mesa-vulkan-drivers \
        pciutils \
        vulkan-tools \
    && rm -rf /var/lib/apt/lists/* \
    # Detect VULKAN_API_VERSION
    && VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` \
    # Create nvidia_icd.json
    && mkdir -p /etc/vulkan/icd.d/ \
    && echo \
"{\n\
    \"file_format_version\" : \"1.0.0\",\n\
    \"ICD\": {\n\
        \"library_path\": \"libGLX_nvidia.so.0\",\n\
        \"api_version\" : \"${VULKAN_API_VERSION}\"\n\
    }\n\
}" > /etc/vulkan/icd.d/nvidia_icd.json

FROM gpu AS gazebo

# Install Gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo* \
    && rm -rf /var/lib/apt/lists/*

FROM gazebo AS fsds

# Install FSDS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    rsync \
    pulseaudio \
    x11-xserver-utils

USER ros

# Clone FSDS repo to home dir (important!) and compile AirSim
WORKDIR /home/ros
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git -b v2.2.0 --recurse-submodules --quiet \
    && ./Formula-Student-Driverless-Simulator/AirSim/setup.sh \
    && ./Formula-Student-Driverless-Simulator/AirSim/build.sh

# Building ros2 bridge
WORKDIR /home/ros/Formula-Student-Driverless-Simulator/ros2
RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install python requrements
WORKDIR /home/ros/Formula-Student-Driverless-Simulator/python
RUN pip3 install -r requirements.txt

# Download and unzip FSDS release
WORKDIR /home/ros/Formula-Student-Driverless-Simulator
RUN rm -f settings.json \
    && wget "https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip" \
    && unzip fsds-v2.2.0-linux.zip \
    && rm fsds-v2.2.0-linux.zip \
    && rm -f settings.json

# fsds_ros2_bridge will read settings from ~/Formula-Student-Driverless-Simulator/settings.json
COPY ./config/settings.json settings.json

USER root

RUN rm -rf /var/lib/apt/lists/*

FROM fsds AS qol

# Set up ROS autocompletion for user
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/* \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/gazebo/setup.bash ]; then source /usr/share/gazebo/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros2/install/setup.bash ]; then source /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros2/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set up aliases
RUN echo 'alias ws="cd ~/ws"' >> /home/$USERNAME/.bashrc \
    && echo 'alias cb="colcon build --symlink-install"' >> /home/$USERNAME/.bashrc \
    && echo 'alias cbd="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug"' >> /home/$USERNAME/.bashrc \
    && echo 'alias cbr="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"' >> /home/$USERNAME/.bashrc \
    && echo 'alias ru="rosdep update"' >> /home/$USERNAME/.bashrc \
    && echo 'alias ri="rosdep install --from-paths src --ignore-src -r -y"' >> /home/$USERNAME/.bashrc \
    && echo 'alias gpu="__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia"' >> /home/$USERNAME/.bashrc \
    && echo 'alias fsds="/home/ros/Formula-Student-Driverless-Simulator/FSDS.sh -WINDOWED -resx=1200"' >> /home/$USERNAME/.bashrc \
    && echo 'alias bridge="source /home/ros/Formula-Student-Driverless-Simulator/ros2/install/setup.bash && ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py"' >> /home/$USERNAME/.bashrc \
    && echo 'alias src="source /home/ros/ws/install/setup.bash && source /home/ros/Formula-Student-Driverless-Simulator/ros2/install/setup.bash"' >> /home/$USERNAME/.bashrc \
    && echo 'alias dv="ros2 launch dv_master_launch dv_master.launch.py"' >> /home/$USERNAME/.bashrc \
    && echo 'alias dvs="ros2 launch dv_master_launch dv_master_sim.launch.py"' >> /home/$USERNAME/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros/ws

ENV DEBIAN_FRONTEND=
