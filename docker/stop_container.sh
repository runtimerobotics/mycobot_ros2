#!/bin/sh

CONTAINER_NAME="mycobot_sim_container" #name of container which you created earlier, by running "create_container.sh" file.

# Running the existing container
docker stop $CONTAINER_NAME
