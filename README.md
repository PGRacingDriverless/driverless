PGRacing Driverless Main Repo
=============================
This is the starting and main repository of the driverless project.
- [Getting started](#Getting-started)
    - [Download](#Download) 
    - [Docker](#Docker)
        - [Build Docker container](#Build-Docker-container)
        - [Run Docker container](#Run-Docker-container)
        - [Container description](#Container-description)
            - [Packages](#Packages)
            - [Aliases](#Aliases)
        - [Extra steps for Windows users](#Extra-steps-for-Windows-users)
            - [Install wsl](#Install-wsl)
            - [Install and config Docker Desktop](#Install-and-config-Docker-Desktop)
- [License](#License)

# Getting started
## Download
Clone or download this repository:
```
git clone https://github.com/PGRacingDriverless/driverless.git
```
Clone repository of your project:
```
cd driverless/
git submodule update --init src/<repo_name>
```
To install submodules of you project use:
```
git submodule update --init --recursive src/<repo_name>
```
## Docker
### Build Docker container
Use the Dockerfile to build the image:
``` bash
cd driverless
./build.sh
```
If you need OpenCV in container use:
```
./build.sh cv
```
### Build Docker container from VS Code
1. Open Visual Studio Code.
2. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
3. Press **Ctrl+Shift+P** and choose **Rebuild and Reopen in container**

### Run Docker container
Run the Docker container (if the container already exists, connect to the existing):
```bash
./run.sh
```

### Run Docker container from VS Code
1. Open Visual Studio Code.
2. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
3. Press **Ctrl+Shift+P** and choose **Reopen in container**
       OR
   Press **Open a Remote Window** button (in the lower left corner blue button with >< icon) and choose **Reopen in container**

Now you will be in container. To open new terminal use **Ctrl+Shift+`**.

### Container description
In container we have:

Working directory is located under path `/home/ros/ws`

#### Packages
Look at Dockerfile under path :
`dirverless/.devcontainer/Dockerfile`

#### Aliases
| Alias | Command |
| ------ | ------ |
|`ws` |`cd ~/ws`|
|`cb` |`colcon build --symlink-install`|
|`cbr`|`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`|
|`rr` |`ros2 run`| 
|`rl` |`ros2 launch`| 
|`ru` |`rosdep update`|
|`ri` |`rosdep install --from-paths src --ignore-src -r -y`|
|`gpu`|`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`| 

### Extra steps for Windows users
#### Install wsl

Every command should be executed in powershell with elevated permissions (right click + run as administrator)

Install wsl2:
```
wsl --install -d Ubuntu
```
Now you can enter wsl by:
```
Ubuntu
```
Or just seach for Ubuntu in search bar
For more info check: https://github.com/microsoft/wslg

#### Install and config Docker Desktop

1. Download and install the latest version of [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/).
2. Follow the usual installation instructions to install Docker Desktop. Depending on which version of Windows you are using, Docker Desktop may prompt you to turn on WSL 2 during installation. Read the information displayed on the screen and turn on the WSL 2 feature to continue.
3. Start Docker Desktop from the Windows Start menu.
4. Navigate to Settings.
5. From the General tab, select Use WSL 2 based engine..
6. If you have installed Docker Desktop on a system that supports WSL 2, this option is turned on by default.
7. Select Apply & Restart.
8. Go to Settings > Resources > WSL Integration.
9. Select "Enable integration with additional distros:" and choose distro you use (UBUNTU)
10. Apply & restart
11. Now docker commands work from Windows using the new WSL 2 engine.

For more info check: https://docs.docker.com/desktop/wsl/.

# License
This project is under an ISC license.
