#!/bin/sh


CONTAINER_NAME="mycobot_sim_container"  # Name of the container you want to create.
DOCKER_IMAGE="mycobot/sim:v0.0.1"  # Name of the Docker image.
HOST_WS_PATH="/home/$USER/mycobot_ws/src"  # Path to your workspace on the host.

xhost +local:docker

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
XAUTH_DOCKER=/tmp/.docker.xauth

# Create Xauth if not present
if [ ! -f "$XAUTH" ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
    else
        touch "$XAUTH"
    fi
    chmod a+r "$XAUTH"
fi

docker rm "$CONTAINER_NAME"

# Check for NVIDIA GPU
if nvidia-smi | grep -q NVIDIA; then
    echo "NVIDIA GPU detected, initializing container with GPU support"
    docker run -it --network host \
        --privileged \
        --name "$CONTAINER_NAME" \
        --gpus all \
        --runtime nvidia \
        -p 27017:27017 \
        -v ~/mongodb7_data:/data/db \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --volume="/etc/timezone:/etc/timezone:ro" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="$XSOCK:$XSOCK:rw" \
        --volume="$XAUTH:$XAUTH_DOCKER:rw" \
        --volume="$HOST_WS_PATH:/home/mycobot/mycobot_ws/src:rw" \
        "$DOCKER_IMAGE" \
        bash
else
    echo "NVIDIA GPU NOT detected, initializing container without GPU support"
    docker run -it --network host \
        --privileged \
        --name "$CONTAINER_NAME" \
        --env="DISPLAY=$DISPLAY" \
        --volume="$XSOCK:$XSOCK:rw" \
        --volume="$XAUTH:$XAUTH_DOCKER:rw" \
        -p 27017:27017 \
        -v ~/mongodb7_data:/data/db \
        --volume="/dev:/dev" \
        --volume="/etc/timezone:/etc/timezone:ro" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="$HOST_WS_PATH:/home/mycobot/mycobot_ws/src:rw" \
        "$DOCKER_IMAGE" \
        bash
fi


