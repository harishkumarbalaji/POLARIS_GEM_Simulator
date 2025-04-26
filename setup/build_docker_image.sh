#!/bin/bash

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: $0 [-f|--force]"
    echo "Build script for simulator Docker Image"
    echo "  -f, --force   Force rebuild without using cache"
    exit 0
fi

BUILD_FLAG=""
if [[ "$1" == "-f" || "$1" == "--force" ]]; then
    BUILD_FLAG="--no-cache"
fi

UID=$(id -u) GID=$(id -g) USER=$(whoami) docker compose -f setup/docker-compose.yaml build $BUILD_FLAG