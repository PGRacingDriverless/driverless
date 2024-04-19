FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04

# Never interact with user
ENV DEBIAN_FRONTEND=noninteractive

# Update and upgrade
RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install base packages
RUN apt-get update && apt-get install -y \
    locales \
    tzdata \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install language
RUN locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata

# Create a non-root user
ARG USERNAME=fsdssim_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Dependencies for glvnd, X11, vulkan
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-toolkit
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV QT_X11_NO_MITSHM=1

# Vulkan support
RUN apt-get update && apt-get install -y --no-install-recommends \ 
    pciutils \
    vulkan-tools \
    mesa-utils \
    libvulkan1 \
    libglu1-mesa-dev \
    mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

# nvidia_icd.json
ARG VULKAN_VERSION="1.3"
RUN VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` && \
    mkdir -p /etc/vulkan/icd.d/ && \
    echo \
"{\n\
    \"file_format_version\" : \"1.0.0\",\n\
    \"ICD\": {\n\
        \"library_path\": \"libGLX_nvidia.so.0\",\n\
        \"api_version\" : \"${VULKAN_API_VERSION}\"\n\
    }\n\
}" > /etc/vulkan/icd.d/nvidia_icd.json

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    lsb-release \
    curl \
    wget \
    nano \
    vim \
    unzip \
    rsync \
    && rm -rf /var/lib/apt/lists/*

# Install dev programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    openssh-client \
    gnupg2 \
    cmake \
    gdb \
    bash-completion \
    python3 \
    python3-pip \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN sudo add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-desktop \
        ros-dev-tools \
        ros-humble-ament-* \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

# Env vars for ROS2
ENV PATH=/opt/ros/humble/bin:$PATH
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages

# Set up autocompletion for user
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends \
    pulseaudio \
    x11-xserver-utils

USER fsdssim_user

# FSDS repo has to be cloned in the home dir
WORKDIR /home/fsdssim_user
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git -b v2.2.0 --recurse-submodules --quiet

# fsds_ros2_bridge will read settings from ~/Formula-Student-Driverless-Simulator/settings.json
COPY ./settings.json Formula-Student-Driverless-Simulator/settings.json

# Copy some camera stuff
# COPY ./files/default.rviz Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/rviz/default.rviz
# COPY ./files/cameralauncher.py Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/scripts/cameralauncher.py
# COPY ./files/fsds_ros_bridge_camera.cpp Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/src/fsds_ros_bridge_camera.cpp

# Compiling the AirSim plugin
WORKDIR /home/fsdssim_user/Formula-Student-Driverless-Simulator/AirSim
RUN ./setup.sh && ./build.sh

# ??? clone race evaluator ???
# RUN cd Formula-Student-Driverless-Simulator/ros/src && \
#     git clone https://github.com/bartoszptak/ros-driving-track-evaluator.git -b sim/fsds --quiet && \
#     git clone https://github.com/PUTvision/RoboticsII-FSDS.git -b ros --quiet && \
#     git clone https://github.com/koide3/ndt_omp.git --quiet && \
#     git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive --quiet && \
#     git clone https://github.com/koide3/hdl_graph_slam --quiet

# Building the ros2 bridge
WORKDIR /home/fsdssim_user/Formula-Student-Driverless-Simulator/ros2
RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# # build ROS bridge
# RUN cd Formula-Student-Driverless-Simulator/ros && \
# 	catkin config --extend /opt/ros/noetic  && \	
#     catkin init && \
# 	LINE=`sed -n '/.publish(go_signal_msg)/=' ./src/fsds_ros_bridge/src/airsim_ros_wrapper.cpp` && \
# 	sed -i "$LINE"' s/^/\/\//' ./src/fsds_ros_bridge/src/airsim_ros_wrapper.cpp && \
#     catkin build

# Set up FSDS autocompletion for user
RUN echo "if [ -f /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros2/install/setup.bash ]; then source /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros2/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# # Add FSDS env to startup
# RUN echo "source /home/fsdssim_user/Formula-Student-Driverless-Simulator/ros2/install/setup.bash" >> ~/.bashrc

# Install python requrements
WORKDIR /home/fsdssim_user/Formula-Student-Driverless-Simulator/python
RUN pip3 install -r requirements.txt

# Download and unzip FSDS release
WORKDIR /home/fsdssim_user/Formula-Student-Driverless-Simulator
RUN rm -rf settings.json
RUN wget "https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip" \
    && unzip fsds-v2.2.0-linux.zip \
    && rm fsds-v2.2.0-linux.zip

USER root

RUN rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=