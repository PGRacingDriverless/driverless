FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04 AS base

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
            libatlas-base-dev \
            libavcodec-dev \
            libavformat-dev \
            libavutil-dev \
            libboost-python-dev \
            libboost-thread-dev \
            libcanberra-gtk3-module \
            libdc1394-dev \
            libeigen3-dev \
            libgl1-mesa-dev \
            libgl1-mesa-glx \
            libglew-dev \
            libglib2.0-0 \
            libgstreamer-plugins-base1.0-dev \
            libgstreamer1.0-dev \
            libgtk-3-dev \
            libgtk2.0-dev \
            libgtkglext1 \
            libgtkglext1-dev \
            libjpeg-dev \
            libjpeg-turbo8-dev \
            libjpeg8-dev \
            liblapack-dev \
            liblapacke-dev \
            libopenblas-dev \
            libopencv-dev \
            libopenexr-dev \
            libpng-dev \
            libpostproc-dev \
            libpq-dev \
            libsm6 \
            libswscale-dev \
            libtbb-dev \
            libtbb2 \
            libtesseract-dev \
            libtiff-dev \
            libtiff5-dev \
            libv4l-dev \
            libvtk9-dev \
            libx11-dev \
            libx264-dev \
            libxext6 \
            libxine2-dev \
            libxrender-dev \
            libxvidcore-dev \
            python3-dev \
            python3-distutils \
            python3-numpy \
            python3-opencv \
            python3-pip \
            python3-pyqt5 \
            python3-setuptools \
        && rm -rf /var/lib/apt/lists/* \
        # Download OpenCV
        && wget -O opencv.zip https://github.com/opencv/opencv/archive/4.9.0.zip \
        && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.9.0.zip \
        # Unzip
        && unzip opencv.zip \
        && unzip opencv_contrib.zip \
        && rm opencv.zip opencv_contrib.zip \
        # Create build directory and switch into it
        && mkdir -p build && cd build \
        # Configure: select the modules you need
        # https://docs.opencv.org/4.x/index.html
        && cmake \
            -DCMAKE_BUILD_TYPE=RELEASE \
            -DWITH_CUDA=NO \
            -DWITH_OPENCL=NO \
            -DBUILD_PERF_TESTS=OFF \
            -DBUILD_TESTS=OFF \
            -DWITH_WIN32UI=OFF \
            -DBUILD_JAVA=OFF \
            -DBUILD_FAT_JAVA_LIB=OFF \
            -DBUILD_OPENCV_PYTHON2=OFF \
            -DBUILD_OPENCV_PYTHON3=OFF \
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
            ../opencv-4.9.0 \
        # Build OpenCV
        && cmake --build . \
    ; fi

# ONNX Runtime
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-x64-1.17.1.tgz \
    # Unzip
    && tar -xvzf onnxruntime-linux-x64-1.17.1.tgz \
    && rm onnxruntime-linux-x64-1.17.1.tgz \
    # Copy ONNX Runtime to /usr/local/
    && cp -R onnxruntime-linux-x64-1.17.1/lib/* /usr/local/lib \
    && cp -R onnxruntime-linux-x64-1.17.1/include/* /usr/local/include \
    && rm -Rf onnxruntime-linux-x64-1.17.1/

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
        # Compilers and other tools to build ROS packages
        ros-dev-tools \
        ros-humble-ament-* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=
