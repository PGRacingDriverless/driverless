#!/bin/bash

readonly DOCKER_GROUP=docker
readonly DOCKER_IMAGE=ros2_pgr_dv
readonly DOCKER_CONTAINER=pgr_dv

readonly CLR_RESET="\033[0m"
readonly CLR_RED="\033[0;31m"

readonly MSG_INACTIVE_DOCKER_DAEMON="${CLR_RED}[ERROR]${CLR_RESET} The docker daemon is not running. Start it using 'sudo systemctl start docker'."
readonly MSG_USER_NOT_IN_GROUP="${CLR_RED}[ERROR]${CLR_RESET} User $USER is not added to the $DOCKER_GROUP group. Add $USER to the $DOCKER_GROUP group or run the script with sudo."
readonly MSG_CONTAINER_RUNNING="[INFO] A container named $DOCKER_CONTAINER is already running. Running bash shell for it."
readonly MSG_CONTAINER_NOT_RUNNING="[INFO] The container named $DOCKER_CONTAINER is not running. Launching it."
readonly MSG_NATIVE_LINUX="[INFO] WSL was not detected. Running as on native Linux."
readonly MSG_WSL="[INFO] WSL distribution of Linux is detected. Run with options for Windows."

is_run_with_sudo() {
	[ "$(id -u)" -eq 0 ]
}

# Returns 0 if WSL is detected and 1 otherwise
# Methods:
# 1. Default kernel name in WSL contains the string "Microsoft" (or "microsoft")
# 2. Check for the automatically injected environment variable $WSL_DISTRO_NAME
is_wsl() {
	if is_run_with_sudo; then
	    [ $(grep -qi microsoft /proc/version) ]
	else
		[ $(grep -qi microsoft /proc/version) ] || [ -n "${WSL_DISTRO_NAME}" ]
	fi
}

# Takes the daemon name as an argument
is_daemon_running() {
	systemctl is-active --quiet $1
}

# Takes the user name and the group name as arguments
is_user_in_group() {
	groups "$1" | grep -qw "$2"
}

# Takes the name of the container as an argument
# Returns container id if running
is_container_running() {
    docker ps -aq -f status=running -f name=$1
}

container_run() {
    if is_wsl; then
        echo $MSG_WSL
		docker run -it \
			--rm \
			-v $PWD/../ws:/home/ros/ws \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v /mnt/wslg:/mnt/wslg \
			-e DISPLAY=:0 \
			-e WAYLAND_DISPLAY=wayland-0 \
			-e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
			-e PULSE_SERVER=/mnt/wslg/PulseServer \
			--gpus=all \
			--ipc=host \
			--name=$1 \
			--network=host \
			--user=ros \
		$2
    else
        echo $MSG_NATIVE_LINUX
		docker run -it \
			--rm \
			-v $PWD/../ws:/home/ros/ws \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			--env=DISPLAY \
			--gpus=all \
			--ipc=host \
			--name=$1 \
			--network=host \
			--user=ros \
		$2
    fi
}

container_exec_bash() {
	docker exec -it $1 bash
}

if is_daemon_running docker; then
	if is_user_in_group $USER $DOCKER_GROUP || is_run_with_sudo; then
		if [ $(is_container_running $DOCKER_CONTAINER) ]; then
			echo $MSG_CONTAINER_RUNNING
			container_exec_bash $DOCKER_CONTAINER
		else
			echo $MSG_CONTAINER_NOT_RUNNING
			container_run $DOCKER_CONTAINER $DOCKER_IMAGE
		fi
	else
		echo -e $MSG_USER_NOT_IN_GROUP
	fi
else
	echo -e $MSG_INACTIVE_DOCKER_DAEMON
fi
