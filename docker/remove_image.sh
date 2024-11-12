#!/bin/sh

CONTAINER_NAME="mycobot_sim_container" #name of container which you created earlier, by running "create_container.sh" file.
IMAGE="mycobot/sim:v0.0.1"
# Running the existing container

docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME
docker rmi $IMAGE
