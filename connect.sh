#!/bin/bash

docker_group_name=docker

docker_exec() {
	docker exec -it pgr_dv bash
}

if groups "$USER" | grep -qw "$docker_group_name"; then
	docker_run
else
	sudo bash -c "$(declare -f docker_exec); docker_exec"
fi
