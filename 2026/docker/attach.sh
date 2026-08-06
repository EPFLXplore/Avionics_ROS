#!/usr/bin/env bash
set -e

# You can override these via env if needed
CONTAINER_NAME=${CONTAINER_NAME:-elec_humble_local}

# NOT ${USERNAME:-xplore}: many desktops export USERNAME, so the host login
# name would leak in and point at /home/<you> inside the container. This is
# the user created in the Dockerfile - hardcoded, same as run.sh.
USERNAME=xplore

echo "Attaching to container: $CONTAINER_NAME"
echo ""

# --------------------------------------------------------
# Basic security / sanity checks
# --------------------------------------------------------

# 1) Check docker is installed
if ! command -v docker >/dev/null 2>&1; then
    echo "Error: 'docker' command not found."
    echo "Make sure Docker is installed and available in your PATH."
    exit 1
fi

# 2) Check Docker daemon is reachable
if ! docker info >/dev/null 2>&1; then
    echo "Error: cannot communicate with the Docker daemon."
    echo "Is Docker running? Try: sudo systemctl start docker"
    exit 1
fi

# 3) Check if container exists
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
    echo "Error: container '${CONTAINER_NAME}' does not exist."
    echo "You probably need to start it first with ./run.sh"
    exit 1
fi

# 4) Check if container is running
CONTAINER_STATE=$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || echo "false")

if [ "$CONTAINER_STATE" != "true" ]; then
    echo "Error: container '${CONTAINER_NAME}' exists but is not running."
    echo "Start it with ./run.sh and then re-run ./attach.sh"
    exit 1
fi

# --------------------------------------------------------
# Exec into the running container
# --------------------------------------------------------
echo "Container is running. Opening an interactive shell..."
echo ""

# The interactive shell reads .bashrc itself, which sets up the ROS env and
# sources the workspace overlay - nothing to source here.
# -w drops you in dev_ws, so colcon is always run from the workspace root.
docker exec -it -w "/home/${USERNAME}/dev_ws" "${CONTAINER_NAME}" /bin/bash

