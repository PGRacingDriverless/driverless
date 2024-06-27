#!/bin/bash

readonly DOCKER_GROUP=docker
readonly DOCKER_IMAGE=pgr\/dev
readonly DOCKER_CONTAINER=dev

if groups "$USER" | grep -qw "$DOCKER_GROUP"; then
	if docker info > /dev/null 2>&1; then
		if [ $(docker ps -aq -f status=running -f name=$DOCKER_CONTAINER) ]; then
			echo "Running bash shell for container named $DOCKER_CONTAINER."
			docker exec -it $DOCKER_CONTAINER bash
		else
			echo "Launching container named $DOCKER_CONTAINER."
			docker run -it \
					--rm \
					--name=$DOCKER_CONTAINER \
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
					$(if grep -qi microsoft /proc/version || [ -n "${WSL_DISTRO_NAME}" ]; then echo "-v /mnt/wslg:/mnt/wslg"; fi) \
					$(if grep -qi microsoft /proc/version || [ -n "${WSL_DISTRO_NAME}" ]; then echo "-e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir"; fi) \
					$(if grep -qi microsoft /proc/version || [ -n "${WSL_DISTRO_NAME}" ]; then echo "-e PULSE_SERVER=/mnt/wslg/PulseServer"; fi) \
					$(if grep -qi microsoft /proc/version || [ -n "${WSL_DISTRO_NAME}" ]; then echo "-e WAYLAND_DISPLAY=wayland-0"; fi) \
				$DOCKER_IMAGE
		fi
	else
		echo "The docker daemon is not running."
	fi
else
	echo "User $USER is not added to the $DOCKER_GROUP group."
fi
