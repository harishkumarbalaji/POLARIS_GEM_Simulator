#!/bin/bash
# Check if container is already running
if [ "$(docker ps -q -f name=ros-noetic-container)" ]; then
    docker exec -it ros-noetic-container bash
else
    export HOST_PATH=${PWD#$HOME}
    export CONTAINER_WD="/home/${USER}/host/${PWD#$HOME/}../../../"
    UID=$(id -u) GID=$(id -g) CONTAINER_WD="$CONTAINER_WD" docker compose -f setup/docker-compose.yaml up -d
    docker exec -it ros-noetic-container bash
fi