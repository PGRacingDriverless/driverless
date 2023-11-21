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
readonly MSG_NOT_RUNNING="[INFO] $(timestamp) The container named $DOCKER_CONTAINER_NAME is not running. Launching it."
readonly MSG_RUNNING="[INFO] $(timestamp) A container named $DOCKER_CONTAINER_NAME is already running. Running bash shell for it."

container_run() {
    docker run -it \
			--rm \
			-v $PWD/../ws:/home/ros/ws \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			--env=DISPLAY \
			--gpus=all \
			--ipc=host \
			--name=pgr_dv \
			--network=host \
			--user=ros \
		ros2_pgr_dv
}

container_exec_bash() {
	docker exec -it pgr_dv bash
}

if groups "$USER" | grep -qw "$DOCKER_GROUP_NAME"; then
	echo $MSG_WITHOUT_SUDO
	
	if [ "$(docker ps -aq -f status=running -f name=$DOCKER_CONTAINER_NAME)" ]; then
		echo $MSG_RUNNING
		container_exec_bash
	else
		echo $MSG_NOT_RUNNING
		container_run
	fi
else
	echo $MSG_WITH_SUDO

	if [ "$(sudo docker ps -aq -f status=running -f name=$DOCKER_CONTAINER_NAME)" ]; then
		echo $MSG_RUNNING
		sudo bash -c "$(declare -f container_exec_bash); container_exec_bash"
	else
		echo $MSG_NOT_RUNNING
		sudo bash -c "$(declare -f container_run); container_run"
	fi
fi