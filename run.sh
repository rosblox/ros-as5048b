#!/bin/bash

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

docker run -it --rm \
--network=host \
--ipc=host --pid=host \
--env UID=$(id -u) \
--env GID=$(id -g) \
--env I2C_GID=$(getent group i2c | cut -d: -f3) \
--device=/dev/i2c-1 \
--env I2C_DEVICE=1 \
--env I2C_ADDRESS=0x40 \
-v /dev:/dev  \
ghcr.io/rosblox/${REPOSITORY_NAME}:humble




#!/bin/bash

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export HOST_UID=$(id -u)

docker compose -f $SCRIPT_DIR/docker-compose.yml run \
--volume ./ros_as5048b:/colcon_ws/src/ros_as5048b \
${REPOSITORY_NAME} bash
