#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONE_XML="${CYCLONE_XML:-$SCRIPT_DIR/cyclonedds.xml}"
CYCLONE_XML_CONTAINER="/etc/cyclonedds/cyclonedds.xml"

CONTAINER_NAME=${CONTAINER_NAME:-elec_humble_local}
IMAGE_NAME=${IMAGE_NAME:-elec:humble-local}
USERNAME=xplore

if [ ! -f "$CYCLONE_XML" ]; then
    echo "error: Cyclone DDS config not found: $CYCLONE_XML" >&2
    exit 1
fi

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
# -w below puts us in dev_ws, so this relative path resolves. The Dockerfile's
# final WORKDIR is /home/xplore, not the workspace, and `bash -c` never reads
# .bashrc - so without -w this line is where the container dies.
DOCKER_COMMAND="source install/setup.bash && { ros2 launch avionics_nexus bridge.launch.py & exec ros2 run python_node python_node; }"

docker run -it \
    --name "$CONTAINER_NAME" \
    -w "/home/$USERNAME/dev_ws" \
    --rm \
    --privileged \
    --net=host \
    -e ROS_DOMAIN_ID=0 \
    -e CYCLONEDDS_URI="file://${CYCLONE_XML_CONTAINER}" \
    -e DISPLAY=unix$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="$XAUTH" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH":"$XAUTH" \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v "$src_dir":/home/$USERNAME/dev_ws/src \
    -v "$SCRIPT_DIR/scripts":/home/$USERNAME/scripts \
    -v "$CYCLONE_XML":"$CYCLONE_XML_CONTAINER":ro \
    -v elec_humble_local_home_volume:/home/$USERNAME \
    "$IMAGE_NAME" \
   /bin/bash -c "$DOCKER_COMMAND"
