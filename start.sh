#!/bin/bash

sudo docker run -it --user ros --gpus all --network=host --ipc=host --name pgr_dv -v $PWD/shared_folder:/shared_folder -v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY ros2_pgr_dv

