#!/usr/bin/env bash
set -e

CONTAINER_NAME=${CONTAINER_NAME:-elec_humble_local}
IMAGE_NAME=${IMAGE_NAME:-elec:humble-local}
# This is the user defined INSIDE your Dockerfile
INTERNAL_USER="xplore"

# ----------------------------------------
# Setup X11 GUI permissions
# ----------------------------------------
XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 2>/dev/null | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f "$XAUTH" ]; then
    if [ -n "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
    else
        touch "$XAUTH"
    fi
    chmod a+r "$XAUTH"
fi

# ----------------------------------------
# Workspace mount logic
# ----------------------------------------
current_dir=$(pwd)
parent_dir=$(dirname "$current_dir")

echo "Mounting workspace:"
echo "  Host:   $parent_dir"
echo "  Target: /home/$INTERNAL_USER/dev_ws/src"
echo ""

# ----------------------------------------
# Run container
# ----------------------------------------
docker run -it \
    --name "$CONTAINER_NAME" \
    --rm \
    --privileged \
    --net=host \
    --entrypoint /bin/bash \
    -e DISPLAY=unix$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="$XAUTH" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH":"$XAUTH" \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v "$parent_dir":/home/$INTERNAL_USER/dev_ws/src \
    -v elec_humble_local_home_volume:/home/$INTERNAL_USER \
    "$IMAGE_NAME" \
    -c "sudo chown -R $INTERNAL_USER:$INTERNAL_USER /home/$INTERNAL_USER; exec /bin/bash"
