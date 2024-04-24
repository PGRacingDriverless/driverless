####################
#       BASE       #
####################
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04 AS base

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
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \    
    software-properties-common \
    lsb-release \
    openssh-client \
    gnupg2 \
    gdb \
    gcc \
    g++ \
    clang \
    make \
    cmake \
    python3 \
    python3-pip \
    python3-argcomplete \
    bash-completion \
    curl \
    git \
    wget \
    tar \
    unzip \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

####################
#       GPU        #
####################
FROM base AS gpu

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

####################
#       ROS2       #
####################
FROM gpu AS ros2

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

####################
#      OpenCV      #
####################
FROM ros2 AS opencv

# Install OpenCV dependencies
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
    libavcodec-dev \
    libavformat-dev \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libdc1394-dev \
    libgtk2.0-dev \
    libjpeg-dev \
    libpng-dev \
    libswscale-dev \
    libtbb-dev \
    libtbb2 \
    libtiff-dev \
    python3-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Download and unzip OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.9.0.zip \
    && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.9.0.zip \
    && unzip opencv.zip \
    && unzip opencv_contrib.zip \
    && rm opencv.zip opencv_contrib.zip

# Create build directory and switch into it
RUN mkdir -p build && cd build
# Configure
# Choose what you need
# https://docs.opencv.org/4.x/index.html
RUN cmake \
    # -DCMAKE_BUILD_TYPE=RELEASE \
    # -DWITH_OPENCL=NO \
    # -DWITH_CUDA=NO \
    # -DBUILD_TESTS=OFF \
    # -DBUILD_PERF_TESTS=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.9.0/modules \
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
    ../opencv-4.9.0
# Build OpenCV
RUN cmake --build .

####################
#       ONNX       #
####################
FROM opencv AS onnx

# Download and unzip ONNX Runtime
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-x64-1.17.1.tgz \
    && tar -xvzf onnxruntime-linux-x64-1.17.1.tgz \
    && rm onnxruntime-linux-x64-1.17.1.tgz
# Copy ONNX Runtime to /usr/local/
RUN cp -R onnxruntime-linux-x64-1.17.1/lib/* /usr/local/lib \
    && cp -R onnxruntime-linux-x64-1.17.1/include/* /usr/local/include \
    && rm -Rf onnxruntime-linux-x64-1.17.1/

####################
#       DEPS       #
####################
FROM onnx AS rosdeps

RUN rosdep init || echo "rosdep already initialized"

# Install rosdeps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-realsense2-camera \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=
