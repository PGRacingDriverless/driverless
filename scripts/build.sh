#!/bin/bash

docker image build -t pgr/dev . \
    --build-arg="OPENCV=$(if [ "$1" == "opencv" ]; then echo true; else echo false; fi)"
