#!/bin/sh
# bash docker-entrypoint_linux.sh

IMAGE=lluisb3/hl2ss:v5.0
IP_HOLOLENS=192.168.1.107
EXP_NAME=hololens_docker

xhost +

sudo docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/docker_volume/hl2ss/data:/home/app/data $IMAGE \
python3 viewer/advanced_recorder_linux.py --exp_name $EXP_NAME --ip_hololens $IP_HOLOLENS

xhost -
