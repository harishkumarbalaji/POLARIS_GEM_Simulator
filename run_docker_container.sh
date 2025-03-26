#!/bin/bash
# Check if container is already running
if [ "$(docker ps -q -f name=ros-noetic-container)" ]; then
    docker exec -it ros-noetic-container bash
else
    UID=$(id -u) GID=$(id -g) docker compose -f setup/docker-compose.yaml up -d
    docker exec -it ros-noetic-container bash
fi