#!/usr/bin/env bash
set -e

CONTAINER_NAME=${CONTAINER_NAME:-elec_humble_local}
IMAGE_NAME=${IMAGE_NAME:-elec:humble-local}
USERNAME=${USERNAME:-xplore}

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

echo "Done."
echo ""
echo "Xauthority file info:"
file "$XAUTH" || true
ls -FAlh "$XAUTH"
echo ""

# ----------------------------------------
# Workspace mount
# ----------------------------------------
current_dir=$(pwd)
src_dir="$current_dir/src"

echo "Mounting workspace:"
echo "  Host: $src_dir"
echo "  ->   /home/$USERNAME/dev_ws/src"
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
    -e ROS_DOMAIN_ID=0 \
    -e DISPLAY=unix$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="$XAUTH" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH":"$XAUTH" \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v "$src_dir":/home/$USERNAME/dev_ws/src \
    -v elec_humble_local_home_volume:/home/$USERNAME \
    "$IMAGE_NAME" \
    -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; exec /bin/bash"

