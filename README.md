# PGRacing Driverless Main Repo
- [Requirements](#Requirements)
- [Guide](#Guide)
    - [Without Docker Compose](#Without-Docker-Compose)
    - [With Docker Compose](#With-Docker-Compose)
    - [With Visual Studio Code](#With-Visual-Studio-Code)
    - [Launch driverless system](#Launch-driverless-system)
- [License](#License)

## Requirements
- Install [Docker Engine](https://docs.docker.com/engine/install/) and [Docker Compose](https://docs.docker.com/compose/install/).
- Install [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) according to the following [guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Guide
### Without Docker Compose
#### Build
```bash
./scripts/build.sh
```
or
```bash
docker image build -t pgr/dev .
```

#### Run
```bash
./scripts/run.sh
```
or
```bash
docker run -it \
        --rm \
        --name=dev \
        --privileged \
        --network=host \
        --ipc=host \
        --pid=host \
        --user=ros \
        --gpus=all \
        -v ./ws:/home/ros/ws \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v /dev/video*:/dev/video* \
        -e DISPLAY=$DISPLAY \
        -e ROS_DOMAIN_ID=5 \
    pgr/dev
```

> **Note**  
> Customize the options to suit your needs.  
> Under WSL you may need additional options such as `-v /mnt/wslg:/mnt/wslg`. The script `run.sh` takes this into account.

### With Docker Compose
#### Build
To build all services:
```bash
docker compose build
```
If you need a specific service, build it with the command:
```bash
docker compose build service_name
```
#### Run
To start a container for a service with an interactive shell:
```bash
docker compose run service_name
```

### With Visual Studio Code
You can also build images and run containers using VS Code.

#### Requirements
- Install [Visual Studio Code](https://code.visualstudio.com/docs/setup/linux).
- Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.

The necessary devcontainer.json files for Dev Containers extension are already in the repository.

#### Build
Inside VS Code, press **Ctrl+Shift+P** and choose **Rebuild and Reopen in container**.
#### Run
Inside VS Code, press **Ctrl+Shift+P** and choose **Reopen in container**.
#### Interact
To open the terminal, use **Ctrl+Shift+`**. 
#### Exit
Inside VS Code, press **Ctrl+Shift+P** and select **Reopen Folder Locally**.

### Launch driverless system
To start an entire driverless system (after building and sourcing):
```bash
ros2 launch dv_master_launch dv_master.launch.py 
```

## License
This project is under an MIT license.
