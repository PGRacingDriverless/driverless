#!/bin/bash

readonly DOCKER_GROUP_NAME=docker
readonly DOCKER_IMAGE_NAME=ros2_pgr_dv
readonly DOCKER_CONTAINER_NAME=pgr_dv

timestamp() {
	date +"%H:%M:%S"
}

readonly CLR_RED="\033[0;31m"
readonly CLR_GREEN="\033[0;32m"
readonly CLR_YELLOW="\033[0;33m"
readonly CLR_END="\033[0m"

readonly MSG_WITH_SUDO="[INFO] $(timestamp) User $USER is not added to the $DOCKER_GROUP_NAME group. Running with sudo."
readonly MSG_WITHOUT_SUDO="[INFO] $(timestamp) User $USER has been added to the $DOCKER_GROUP_NAME group. Running without sudo."
readonly MSG_RUNNING="[INFO] $(timestamp) A container named $DOCKER_CONTAINER_NAME is already running. Running bash shell for it."
readonly MSG_NOT_RUNNING="[INFO] $(timestamp) The container named $DOCKER_CONTAINER_NAME is not running. Launching it."

container_run() {
	readonly MSG_NATIVE_LINUX="[INFO] `date +"%H:%M:%S"` WSL was not detected. Running as on native Linux."
	readonly MSG_WSL_LINUX="[INFO] `date +"%H:%M:%S"` WSL distribution of Linux is detected. Run with options for Windows."

	if grep -qi microsoft /proc/version; then
		echo $MSG_WSL_LINUX
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

if groups "$USER" | grep -qw "$DOCKER_GROUP_NAME"; then
	echo $MSG_WITHOUT_SUDO
	
	if [ "$(docker ps -aq -f status=running -f name=$DOCKER_CONTAINER_NAME)" ]; then
		echo $MSG_RUNNING
		container_exec_bash $DOCKER_CONTAINER_NAME
	else
		echo $MSG_NOT_RUNNING
		container_run $DOCKER_CONTAINER_NAME $DOCKER_IMAGE_NAME
	fi
else
	echo $MSG_WITH_SUDO

	if [ "$(sudo docker ps -aq -f status=running -f name=$DOCKER_CONTAINER_NAME)" ]; then
		echo $MSG_RUNNING
		sudo bash -c "$(declare -f container_exec_bash); container_exec_bash $DOCKER_CONTAINER_NAME"
	else
		echo $MSG_NOT_RUNNING
		sudo bash -c "$(declare -f container_run); container_run $DOCKER_CONTAINER_NAME $DOCKER_IMAGE_NAME"
	fi
fi