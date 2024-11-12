#!/usr/bin/env bash
set -e

# setup ros environment
source "/home/mycobot/mycobot_ws/install/setup.bash"

exec "$@"