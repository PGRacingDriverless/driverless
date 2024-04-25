#!/bin/bash

if [ "$1" = "opencv" ]; then
    docker image build -t pgr/dv . --build-arg="OPENCV=true"
else
    docker image build -t pgr/dv .
fi
