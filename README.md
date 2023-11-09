# PGRacing Driverless Main Repo
This is the starting and main repository of the driverless project.

Code style: https://github.com/PGRacingDriverless/driverless/edit/master/README.md

### How to get started?
Clone or download the repository:
```
git clone https://github.com/PGRacingDriverless/driverless.git
```
Use the Dockerfile to build the image:
```
cd driverless
docker image build -t ros2_pgr_dv .
```
Run the Docker container from the image:
```
docker run -it --rm \
    -v $PWD/shared_folder:/shared_folder \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    --gpus=all \
    --ipc=host \
    --name=pgr_dv \
    --network=host \
    --user=ros \
    ros2_pgr_dv
```
Or simply run through start script:
```
./start.sh
```

You can customize the options you need. A table of some of the options is shown below.
| Option | Interpretation |
| ------ | ------ |
| ```-p 8080:80``` | Map port 8080 on the Docker host to TCP port 80 in the container. |
| ```-v /dev:/dev``` | Device forwarding. Used with ```--device-cgroup-rule``` or ```--device```. |
| ```-v $PWD/shared_folder:/shared_folder``` | Set up a shared folder for the host machine and docker container. |
| ```-v /tmp/.X11-unix:/tmp/.X11-unix:rw``` | X11 forwarding (read-write) for GUI apps running in Docker. The ```--env``` option is required. ```$ xhost +local:``` command can be required. |
| ```--rm``` | Automatically clean up the container when the container exits. Also removes the associated anonymous volumes. |
| ```--device-cgroup-rule='c *:* rmw'``` | Add a rule to the cgroup allowed devices list. |
| ```--env=DISPLAY``` | Sets the current display. |
| ```--gpus=all``` | Аccess to all GPU resources for the container. [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) or [Nvidia Container Runtime](https://docs.docker.com/config/containers/resource_constraints/#install-nvidia-container-runtime) (deprecated) is required. |
| ```--ipc=host``` | Inter-process communication (higher performance for complex computations). |
| ```--name=<your_name>``` | Sets the container name. |
| ```--network=host``` | Networking using the host network. |
| ```--user=ros``` | Run under the ros user (for dev purposes). Otherwise will run under root. |

### How to interact with an existing Docker container from another console window?
To execute a command in an existing container:
```
docker exec -it pgr_dv <command>
```
A new bash console for an existing container:
```
docker exec -it pgr_dv bash
```
Or simply run through start script:
```
./connect.sh
```

### How to run an application with a GPU in a Docker container?
You can launch an application with a GPU in a container (if the necessary options are set) by executing the following command:
```
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia <app_to_execute>
```

### How to connect devices to a Docker container?
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
3. Serial device (```RUN usermod -aG dialout ${USERNAME}``` in Dockerfile is required)
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

## License
???

