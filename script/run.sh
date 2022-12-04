#!/bin/bash
# docker run -u=$(id -u $USER):$(id -g $USER) -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $(pwd)/app:/app --rm -it python-docker:latest bash
docker run --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --rm -it --net=host python-docker:latest bash

# xhost +local:root # for the lazy and reckless
# xhost -local:root
