#!/bin/bash

docker_group_name=docker

docker_run() {
    docker run -it \
			--rm \
			-v $PWD/ws:/home/ros/ws \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			--env=DISPLAY \
			--gpus=all \
			--ipc=host \
			--name=pgr_dv \
			--network=host \
			--user=ros \
		ros2_pgr_dv
}

if groups "$USER" | grep -qw "$docker_group_name"; then
	docker_run
else
	sudo bash -c "$(declare -f docker_run); docker_run"
fi

