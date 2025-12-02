#!/usr/bin/env bash
set -e

# You can override these via env if needed
CONTAINER_NAME=${CONTAINER_NAME:-elec_humble_local}
USERNAME=${USERNAME:-xplore}

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

# Source user's .bashrc inside the container so ROS/micro-ROS env is loaded
docker exec -it "${CONTAINER_NAME}" /bin/bash -c "source /home/${USERNAME}/.bashrc; exec /bin/bash"

