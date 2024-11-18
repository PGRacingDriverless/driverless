FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04 AS base

# Never interact with user
ENV DEBIAN_FRONTEND=noninteractive

# Upgrade packages
RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install packages
RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    clang \
    cmake \
    curl \
    g++ \
    gcc \
    gdb \
    git \
    gnupg2 \
    locales \
    lsb-release \
    make \
    nano \
    pkg-config \
    python3 \
    python3-pip \
    software-properties-common \
    sudo \
    tar \
    tzdata \
    unzip \
    vim \
    wget \
&& rm -rf /var/lib/apt/lists/*

# Install language
RUN locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

FROM base AS ros2

# Install ROS2 Humble
RUN sudo add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        # Base install (Bare Bones)
        ros-humble-ros-base \
        # ROS 2 command line tools use argcomplete to autocompletion
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/* \
    # Set up ROS autocompletion for user
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc

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

FROM ros2 AS camera

# OpenCV
# Use --build-arg="OPENCV=true" to build with OpenCV
ARG OPENCV=false
RUN if [ "$OPENCV" = "true" ]; then \
        # Install OpenCV dependencies
        apt-get update && apt-get install -y -qq --no-install-recommends \
            # OpenCV dependencies
            libgtk2.0-dev \
            libavcodec-dev \
            libavformat-dev \
            libswscale-dev \
            python3-dev \
            python3-numpy \
            libtbb2 \
            libtbb-dev \
            libjpeg-dev \
            libpng-dev \
            libtiff-dev \
            libdc1394-dev \
            libcanberra-gtk-module \
            libcanberra-gtk3-module \
        && rm -rf /var/lib/apt/lists/* \
        # Download OpenCV
        && wget -O opencv.zip https://github.com/opencv/opencv/archive/4.9.0.zip \
        && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.9.0.zip \
        # Unzip
        && unzip opencv.zip \
        && unzip opencv_contrib.zip \
        && rm opencv.zip opencv_contrib.zip \
        # Create build directory and switch into it
        && cd /opencv-4.9.0 && mkdir -p build && cd build \
        # Configure: select the modules you need
        # https://docs.opencv.org/4.x/index.html
        && cmake \
            -DCMAKE_BUILD_TYPE=RELEASE \
            -DCMAKE_INSTALL_PREFIX=/usr/local \
            -DWITH_CUDA=NO \
            -DWITH_OPENCL=NO \
            -DBUILD_PERF_TESTS=OFF \
            -DBUILD_TESTS=OFF \
            -DWITH_WIN32UI=OFF \
            -DBUILD_JAVA=OFF \
            -DBUILD_FAT_JAVA_LIB=OFF \
            -DBUILD_OPENCV_PYTHON2=OFF \
            -DBUILD_OPENCV_PYTHON3=OFF \
            -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.9.0/modules \
                -DBUILD_opencv_alphamat=OFF \
                -DBUILD_opencv_aruco=OFF \
                -DBUILD_opencv_bgsegm=OFF \
                -DBUILD_opencv_bioinspired=OFF \
                -DBUILD_opencv_cannops=OFF \
                -DBUILD_opencv_ccalib=OFF \
                -DBUILD_opencv_cudaarithm=OFF \
                -DBUILD_opencv_cudabgsegm=OFF \
                -DBUILD_opencv_cudacodec=OFF \
                -DBUILD_opencv_cudafeatures2d=OFF \
                -DBUILD_opencv_cudafilters=OFF \
                -DBUILD_opencv_cudaimgproc=OFF \
                -DBUILD_opencv_cudalegacy=OFF \
                -DBUILD_opencv_cudaobjdetect=OFF \
                -DBUILD_opencv_cudaoptflow=OFF \
                -DBUILD_opencv_cudastereo=OFF \
                -DBUILD_opencv_cudawarping=OFF \
                -DBUILD_opencv_cudev=OFF \
                -DBUILD_opencv_cvv=OFF \
                -DBUILD_opencv_datasets=OFF \
                -DBUILD_opencv_dnn_objdetect=OFF \
                -DBUILD_opencv_dnn_superres=OFF \
                -DBUILD_opencv_dpm=OFF \
                -DBUILD_opencv_face=OFF \
                -DBUILD_opencv_freetype=OFF \
                -DBUILD_opencv_fuzzy=OFF \
                -DBUILD_opencv_hdf=OFF \
                -DBUILD_opencv_hfs=OFF \
                -DBUILD_opencv_img_hash=OFF \
                -DBUILD_opencv_intensity_transform=OFF \
                -DBUILD_opencv_julia=OFF \
                -DBUILD_opencv_line_descriptor=OFF \
                -DBUILD_opencv_mcc=OFF \
                -DBUILD_opencv_optflow=OFF \
                -DBUILD_opencv_ovis=OFF \
                -DBUILD_opencv_phase_unwrapping=OFF \
                -DBUILD_opencv_plot=OFF \
                -DBUILD_opencv_quality=OFF \
                -DBUILD_opencv_rapid=OFF \
                -DBUILD_opencv_reg=OFF \
                -DBUILD_opencv_rgbd=OFF \
                -DBUILD_opencv_saliency=OFF \
                -DBUILD_opencv_sfm=OFF \
                -DBUILD_opencv_shape=OFF \
                -DBUILD_opencv_signal=OFF \
                -DBUILD_opencv_stereo=OFF \
                -DBUILD_opencv_structured_light=OFF \
                -DBUILD_opencv_superres=OFF \
                -DBUILD_opencv_surface_matching=OFF \
                -DBUILD_opencv_text=OFF \
                -DBUILD_opencv_tracking=OFF \
                -DBUILD_opencv_videostab=OFF \
                -DBUILD_opencv_viz=OFF \
                -DBUILD_opencv_wechat_qrcode=OFF \
                -DBUILD_opencv_xfeatures2d=OFF \
                -DBUILD_opencv_ximgproc=OFF \
                -DBUILD_opencv_xobjdetect=OFF \
                -DBUILD_opencv_xphoto=OFF \
            .. \
        # Build OpenCV
        && cmake --build . \
        # Install OpenCV
        && make -j"$(nproc)" \
        && make install \
        # Clear
        && cd / \
        && rm -rf /opencv-4.9.0 /opencv_contrib-4.9.0 \
    ; fi

RUN pip install ultralytics

# ONNX Runtime
# Note: You can change LD_LIBRARY_PATH / LIBRARY_PATH
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-x64-gpu-cuda12-1.18.1.tgz \
    && wget https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-x64-1.18.1.tgz \
    # Unzip
    && tar -xvzf onnxruntime-linux-x64-gpu-cuda12-1.18.1.tgz \
    && tar -xvzf onnxruntime-linux-x64-1.18.1.tgz \
    # Copy ONNX Runtime to /usr/local/
    && mkdir -p /usr/local/onnxruntime-libs \
    && mv onnxruntime-linux-x64-gpu-1.18.1 onnxruntime-linux-x64-1.18.1 /usr/local/onnxruntime-libs/ \
    # Clear
    && rm onnxruntime-linux-x64-gpu-cuda12-1.18.1.tgz \
    && rm onnxruntime-linux-x64-1.18.1.tgz

FROM camera AS rosdeps

RUN rosdep init || echo "" \
    # Install dep packages
    && apt-get update && apt-get install -y --no-install-recommends \
        # camera
        ros-humble-cv-bridge \
        ros-humble-realsense2-camera \
        ros-humble-vision-opencv \
        # ???
        ros-humble-pcl-ros \
        ros-humble-cartographer-ros \
        ros-humble-hls-lfcd-lds-driver \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-gtsam \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        # Desktop Install: ROS, RViz, demos, tutorials
        ros-humble-desktop \
        # Compilers and other tools to build ROS packages
        ros-dev-tools \
        ros-humble-ament-* \
    && rm -rf /var/lib/apt/lists/*

FROM rosdeps AS gpu
# Expose Nvidia driver to allow OpenGL
# Dependencies for GLVND, X11, Vulkan
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
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
    && rm -rf /var/lib/apt/lists/* \
    # Set up Gazebo for user
    && echo "if [ -f /usr/share/gazebo/setup.bash ]; then source /usr/share/gazebo/setup.bash; fi" >> /home/$USERNAME/.bashrc

FROM gazebo AS fsds

RUN apt-get update && apt-get install -y --no-install-recommends \
    rsync \
    pulseaudio \
    x11-xserver-utils

USER ros

# FSDS repo has to be cloned in the home dir
WORKDIR /home/ros
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git -b v2.2.0 --recurse-submodules --quiet \
    # Compiling the AirSim plugin
    && ./Formula-Student-Driverless-Simulator/AirSim/setup.sh \
    && ./Formula-Student-Driverless-Simulator/AirSim/build.sh

# Building the ros2 bridge
WORKDIR /home/ros/Formula-Student-Driverless-Simulator/ros2
RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install python requrements
WORKDIR /home/ros/Formula-Student-Driverless-Simulator/python
RUN pip3 install -r requirements.txt

# Download and unzip FSDS release
WORKDIR /home/ros/Formula-Student-Driverless-Simulator
RUN rm -rf settings.json \
    && wget "https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip" \
    && unzip fsds-v2.2.0-linux.zip \
    && rm fsds-v2.2.0-linux.zip \
    && rm -rf settings.json

# fsds_ros2_bridge will read settings from ~/Formula-Student-Driverless-Simulator/settings.json
COPY ./config/settings.json settings.json

# Copy some camera stuff
# COPY ./files/default.rviz Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/rviz/default.rviz
# COPY ./files/cameralauncher.py Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/scripts/cameralauncher.py
# COPY ./files/fsds_ros_bridge_camera.cpp Formula-Student-Driverless-Simulator/ros/src/fsds_ros_bridge/src/fsds_ros_bridge_camera.cpp

# Set up FSDS autocompletion for user
RUN echo "if [ -f /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros2/install/setup.bash ]; then source /home/${USERNAME}/Formula-Student-Driverless-Simulator/ros2/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

USER root

RUN rm -rf /var/lib/apt/lists/*

FROM fsds AS aliases

# Set up aliases
RUN echo 'alias ws="cd ~/ws"' >> /home/$USERNAME/.bashrc \
    && echo 'alias cb="colcon build --symlink-install"' >> /home/$USERNAME/.bashrc \
    && echo 'alias cbr="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"' >> /home/$USERNAME/.bashrc \
    && echo 'alias ru="rosdep update"' >> /home/$USERNAME/.bashrc \
    && echo 'alias ri="rosdep install --from-paths src --ignore-src -r -y"' >> /home/$USERNAME/.bashrc \
    && echo 'alias gpu="__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia"' >> /home/$USERNAME/.bashrc

WORKDIR /home/ros/ws

ENV DEBIAN_FRONTEND=
