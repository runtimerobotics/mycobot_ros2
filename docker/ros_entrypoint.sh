#!/usr/bin/env bash
set -e

mongod --fork --logpath /var/log/mongodb/mongod.log --dbpath /data/db


# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

exec "$@"