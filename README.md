# PGRacing Driverless Main Repo
This is the starting and main repository of the driverless project.

## How to get started?
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
sudo docker run -it \
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
You can customize the options and arguments you need. Tables of some of the options and arguments is shown below.
| Option | Interpretation |
| ------ | ------ |
| ```-p 8080:80``` | Map port 8080 on the Docker host to TCP port 80 in the container. |
| ```-v /dev:/dev``` | Device forwarding. Used with ```--device-cgroup-rule``` or ```--device```. |
| ```-v $PWD/shared_folder:/shared_folder``` | Set up a shared folder for the host machine and docker container. |
| ```-v /tmp/.X11-unix:/tmp/.X11-unix:rw``` | X11 forwarding (read-write) for GUI apps running in Docker. The ```--env``` argument is required. ```$ xhost +local:``` command can be required. |

| Argument | Interpretation |
| ------ | ------ |
| ```--device-cgroup-rule='c *:* rmw'``` | Add a rule to the cgroup allowed devices list. |
| ```--env=DISPLAY``` | Sets the current display. |
| ```--gpus=all``` | Аccess to all GPU resources for the container. [Nvidia-container-runtime](https://docs.docker.com/config/containers/resource_constraints/#install-nvidia-container-runtime) is required. |
| ```--ipc=host``` | Inter-process communication (higher performance for complex computations). |
| ```--name=<your_name>``` | Sets the container name. |
| ```--network=host``` | Networking using the host network. |
| ```--user=ros``` | Run under the ros user (for dev purposes). Otherwise will run under root. |

## License
???

