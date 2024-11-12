#!/bin/bash
# This script is to create a docker image from the Dockerfile.

# Set the image name as a variable here
IMAGE_NAME="mycobot/sim:v0.0.1"
ROS_DISTRO=$1

# Check if the current directory is inside the "src/mycobot_ros2" folder
if [[ "$PWD" == */src/mycobot_ros2* ]]; then
    echo "Current directory is inside the src/mycobot folder, proceeding with Docker build..."

    # Build the Docker image with the specified image name
    docker build -f docker/Dockerfile.mycobot_sim.$ROS_DISTRO -t "$IMAGE_NAME" .
else
    echo "Error: This script must be run from a directory inside the 'src/mycobot_ros2' folder of your workspace. Then run ./docker/build_image.sh"
    exit 1
fi
