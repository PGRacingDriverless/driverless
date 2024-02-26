# PGRacing Driverless Main Repo
This is the starting and main repository of the driverless project.
- [Getting started](#Getting-started)
    - [Download](#Download) 
    - [Build Docker container](#Build-Docker-container)
    - [Run Docker container](#Run-Docker-container)
- [Aliases](#Aliases)
- [License](#License)

## Getting started
### Download
Clone or download the repository:
```bash
git clone https://github.com/PGRacingDriverless/driverless.git
```
Clone your subproject repository as follows:
```bash
cd driverless/ws/src
git clone <your_repo>
```

**OR** by using a submodule:
```bash
cd driverless
git submodule update --init ws/src/<your_repo>
```
To clone your subproject as a submodule containing other submodules, use:
```bash
cd driverless
git submodule update --init --recursive ws/src/<your_repo>
```

### Build Docker container
Use the script to build the image from Dockerfile:
```bash
cd driverless
./build.sh
```
If you need the OpenCV library in a container, use with argument:
```bash
./build.sh opencv
```
> **Important**  
> Build with opencv takes significantly more time.

**OR** you can also build the image with Visual Studio Code:
1. Open Visual Studio Code.
2. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
3. Press **Ctrl+Shift+P** and choose **Rebuild and Reopen in container**.

### Run Docker container
> **IMPORTANT**  
> To run GPU containers with Docker, install the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) using the following [guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Run the container with a script:
```bash
./run.sh
```
If the container is already running, the script will connect you to it.

**OR** with Visual Studio Code:
1. Open Visual Studio Code.
2. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
3. Press **Ctrl+Shift+P** and choose **Reopen in container**.

Now you will be in a container. To open a new terminal, use **Ctrl+Shift+`**.  
To exit the container, press **Ctrl+Shift+P** and select **Reopen Folder Locally**.

## Aliases
The following aliases are defined inside the container:
| Alias | Command |
| ------ | ------ |
| `ws` | `cd ~/ws` |
| `cb` | `colcon build --symlink-install` |
| `cbr` | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` |
| `rr` | `ros2 run` |
| `rl` | `ros2 launch` |
| `ru` | `rosdep update` |
| `ri` | `rosdep install --from-paths src --ignore-src -r -y` |
| `gpu` | `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` |

## License
This project is under an ISC license.
