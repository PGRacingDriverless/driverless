PGRacing Driverless Main Repo
=============================
This is the starting and main repository of the driverless project.
- [Getting started](#Getting-started)
    - [Docker](#Docker)
        - [Build Docker container](#Build-Docker-container)
        - [Run Docker container](#Run-Docker-container)
        - [Interacting with an existing Docker container from another console window](#Interacting-with-an-existing-Docker-container-from-another-console-window)
        - [Running applications on GPU in a Docker container](#Running-applications-on-GPU-in-a-Docker-container)
    - [Robot Operating System](#Robot-Operating-System)
        - [Materials](#Materials)
- [More about Docker](#More-about-Docker)
    - [Docker run options](#Docker-run-options)
    - [Devices in Docker](#Devices-in-Docker)
- [Troubleshooting](#Troubleshooting)
    - [Run Linux GUI apps on the Windows Subsystem for Linux](#Run-Linux-GUI-apps-on-the-Windows-Subsystem-for-Linux)
    - [Authorization problem for GUI apps](#Authorization-problem-for-GUI-apps)
- [License](#License)

# Getting started
## Docker
### Build Docker container
Clone or download the repository:
```
git clone https://github.com/PGRacingDriverless/driverless.git
```
Use the Dockerfile to build the image:
```
cd driverless
docker image build -t ros2_pgr_dv .
```
### Run Docker container
Run the Docker container from the image:
```
docker run -it --rm \
    -v $PWD/ws:/home/ros/ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    --gpus=all \
    --ipc=host \
    --name=pgr_dv \
    --network=host \
    --user=ros \
    ros2_pgr_dv
```
You can customize the options you need. [Here](#Docker-run-options) is a table of some of the options useful within this project.  
If you are a Windows user, it will be helpful to refer to the [troubleshooting](#Troubleshooting) section.
### Interacting with an existing Docker container from another console window
To execute a command in an existing container:
```
docker exec -it pgr_dv <command>
```
A new bash console for an existing container:
```
docker exec -it pgr_dv bash
```
### Running applications on GPU in a Docker container
You can run an application on the GPU in a container (if the necessary [options](#Options) are set) by executing the following command:
```
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia <app_to_execute>
```
## Robot Operating System
Everything needed has been added to the Dockerfile, so ROS2 (Humble version) will already be installed in the built Docker image.
### Materials
[Docs](https://docs.ros.org/en/humble/index.html)

# More about Docker
## Docker run options
| Option | Interpretation |
| ------ | ------ |
| ```-p 8080:80``` | Map port 8080 on the Docker host to TCP port 80 in the container. |
| ```-v /dev:/dev``` | Device forwarding. Used with ```--device-cgroup-rule``` or ```--device```. |
| ```-v $PWD/ws:/home/ros/ws``` | Set up a shared folder for the host machine and docker container. |
| ```-v /tmp/.X11-unix:/tmp/.X11-unix:rw``` | X11 forwarding (read-write) for GUI apps running in Docker. The ```--env``` option is required. |
| ```--rm``` | Automatically clean up the container when the container exits. Also removes the associated anonymous volumes. |
| ```--device-cgroup-rule='c *:* rmw'``` | Add a rule to the cgroup allowed devices list. |
| ```--env=DISPLAY``` | Sets the current display. |
| ```--gpus=all``` | Аccess to all GPU resources for the container. [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) or [Nvidia Container Runtime](https://docs.docker.com/config/containers/resource_constraints/#install-nvidia-container-runtime) (deprecated) is required. |
| ```--ipc=host``` | Inter-process communication (higher performance for complex computations). |
| ```--name=<your_name>``` | Sets the container name. |
| ```--network=host``` | Networking using the host network. |
| ```--user=ros``` | Run under the ros user (for dev purposes). Otherwise will run under root. |
## Devices in Docker
Common case for all devices:
```
docker run -it \
    -v /dev:/dev \
    --device-cgroup-rule='c *:* rmw' \
    ros2_pgr_dv
```

Particular cases for:
1. Joystick
```
docker run -it \
    -v /dev/input:/dev/input \
    --device-cgroup-rule='c 13:* rmw' \
    ros2_pgr_dv
```
2. Depth camera
```
docker run -it \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmv' \
    ros2_pgr_dv ros2 launch depthai_examples rgb_stereo_node.launch.py
```
3. Serial device (??? ```RUN usermod -aG dialout ${USERNAME}``` in Dockerfile is required ???)
```
docker run -it \
    -v /dev:/dev \
    --device-cgroup-rule='c 166:1 rmw' \
    ros2_pgr_dv
```
or
```
docker run -it \
    -v /dev:/dev \
    --device=/dev/ttyACM1
    ros2_pgr_dv
```
# Troubleshooting
## Run Linux GUI apps on the Windows Subsystem for Linux
### Error
...
### Problem
1. The container host (i.e. docker-desktop-data WSL2 distribution) does not have a ```/tmp/.X11-unix``` itself. This folder is actually found in the ```/mnt/host/wslg/.X11-unix``` folder on the docker-desktop distribution which translates to ```/run/desktop/mnt/host/wslg/.X11-unix``` when running containers.
2. There are no baked-in environment variables to assist you, so you need to specify the environment variables explicitly with these folders in mind.
### Solution
[Run Linux GUI apps on the Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)  
[Containerizing GUI applications with WSLg](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)  
[How to show GUI apps from docker desktop container on windows 11](https://stackoverflow.com/questions/73092750/how-to-show-gui-apps-from-docker-desktop-container-on-windows-11)  
## Authorization problem for GUI apps
### Error
All the necessary options for GUI are set, but the following error appears when running GUI applications:
```Authorization required, but no authorization protocol specified```
### Problem
Applications cannot contact your X11 display because they do not have permission to do so because they are running as a different user.
### Solution
Enter the ```$ xhost +local:``` command on the host machine to allow other users to run programs in your session (network connections will not be allowed in this case).
Use the ```$ xhost +``` command if you want to allow clients from any host (unsafe).

# License
In the process of clarifying...
