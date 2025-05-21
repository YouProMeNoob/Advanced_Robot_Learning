#!/bin/bash

xhost +local:root
echo "Starting a new container..."
#docker rm -f om_noetic_v2 /dev/null || true

docker start om_noetic_v2 
    

