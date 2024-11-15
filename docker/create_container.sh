#!/bin/sh

CONTAINER_NAME="mycobot_sim_container"  # Name of the container to create.
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

# Remove existing container
docker rm -f "$CONTAINER_NAME"

BUS=$(lsusb | grep 8086 | cut -d " " -f 2)
PORT=$(lsusb | grep 8086 | cut -d " " -f 4 | cut -d ":" -f 1)

# Check for NVIDIA GPU
if nvidia-smi | grep -q NVIDIA; then
    echo "NVIDIA GPU detected, initializing container with GPU support"
    if [ ! -z "$PORT" ]; then
        echo "NVIDIA with RealSense"
        docker run -it --network host \
            --privileged \
            --name "$CONTAINER_NAME" \
            --gpus all \
            --runtime nvidia \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --env="NVIDIA_DRIVER_CAPABILITIES=all" \
            --volume="/etc/timezone:/etc/timezone:ro" \
            --volume="/etc/localtime:/etc/localtime:ro" \
            --volume="/dev/bus/usb/$BUS/$PORT:/dev/bus/usb/$BUS/$PORT" \
            --device-cgroup-rule "c 189:* rmw" \
            --volume="$XSOCK:$XSOCK:rw" \
            --volume="$XAUTH:$XAUTH_DOCKER:rw" \
            --volume="$HOST_WS_PATH:/home/mycobot/mycobot_ws/src:rw" \
            "$DOCKER_IMAGE" \
            bash
    else
        echo "NVIDIA without RealSense"
        docker run -it --network host \
            --privileged \
            --name "$CONTAINER_NAME" \
            --gpus all \
            --runtime nvidia \
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
    fi
else
    echo "NVIDIA GPU NOT detected, initializing container without GPU support"
    if [ ! -z "$PORT" ]; then
        echo "No NVIDIA with RealSense"
        docker run -it --network host \
            --privileged \
            --name "$CONTAINER_NAME" \
            --env="DISPLAY=$DISPLAY" \
            --volume="$XSOCK:$XSOCK:rw" \
            --volume="$XAUTH:$XAUTH_DOCKER:rw" \
            --volume="/etc/timezone:/etc/timezone:ro" \
            --volume="/etc/localtime:/etc/localtime:ro" \
            --volume="/dev/bus/usb/$BUS/$PORT:/dev/bus/usb/$BUS/$PORT" \
            --device-cgroup-rule "c 189:* rmw" \
            --volume="$HOST_WS_PATH:/home/mycobot/mycobot_ws/src:rw" \
            "$DOCKER_IMAGE" \
            bash
    else
        echo "No NVIDIA with No RealSense"
        docker run -it --network host \
            --privileged \
            --name "$CONTAINER_NAME" \
            --env="DISPLAY=$DISPLAY" \
            --volume="$XSOCK:$XSOCK:rw" \
            --volume="$XAUTH:$XAUTH_DOCKER:rw" \
            --volume="/etc/timezone:/etc/timezone:ro" \
            --volume="/etc/localtime:/etc/localtime:ro" \
            --volume="$HOST_WS_PATH:/home/mycobot/mycobot_ws/src:rw" \
            "$DOCKER_IMAGE" \
            bash
    fi
fi
