#!/bin/bash

xhost +local:root
echo "Starting a new container..."
docker rm -f om_noetic_v2 || true

docker run -it --name om_noetic_v2 \
    --device=/dev/ttyUSB0 \
    --device=/dev/video2 \
    --device=/dev/video3 \
    --device=/dev/video4 \
    --device=/dev/video5 \
    --device=/dev/video6 \
    --device=/dev/video7 \
    --device=/dev/bus/usb \
    --group-add video \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --device-cgroup-rule="c 81:* rmw" \
    --device-cgroup-rule="c 189:* rmw" \
    --net=host \
    --privileged \
    temp
    

