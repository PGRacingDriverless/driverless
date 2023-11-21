#!/bin/bash

readonly DOCKER_GROUP_NAME=docker
readonly DOCKER_IMAGE_NAME=ros2_pgr_dv
readonly DOCKER_CONTAINER_NAME=pgr_dv

timestamp() {
	date +"%H:%M:%S"
}

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

if groups "$USER" | grep -qw "$DOCKER_GROUP_NAME";
then
	echo "[INFO]: $(timestamp)"\
	"User $USER has been added to the $DOCKER_GROUP_NAME group."\
	"Running without sudo."
	
	if [ "$(docker ps -aq -f status=running -f name=$DOCKER_CONTAINER_NAME)" ];
	then
		echo "[INFO]: $(timestamp)"\
		"A container named $DOCKER_CONTAINER_NAME is already running."\
		"Running bash shell for it."

		container_exec_bash
	else
		echo "[INFO]: $(timestamp)"\
		"The container named $DOCKER_CONTAINER_NAME is not running. Launching it."

		container_run
	fi
else
	echo "[INFO]: $(timestamp)"\
	"User $USER is not added to the $DOCKER_GROUP_NAME group."\
	"Running with sudo."
	
	if [ "$(sudo docker ps -aq -f status=running -f name=$DOCKER_CONTAINER_NAME)" ];
	then
		echo "[INFO]: $(timestamp)"\
		"A container named $DOCKER_CONTAINER_NAME is already running."\
		"Running bash shell for it."

		sudo bash -c "$(declare -f container_exec_bash); container_exec_bash"
	else
		echo "[INFO]: $(timestamp)"\
		"The container named $DOCKER_CONTAINER_NAME is not running. Launching it."

		sudo bash -c "$(declare -f container_run); container_run"
	fi
fi