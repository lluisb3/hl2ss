#!/bin/sh
# For Ubuntu devices #
# bash script to create image mapping host user's ID with non-root user in the container #

IMAGE=lluisb3/hl2ss:v9.0

docker build --no-cache --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t $IMAGE .
