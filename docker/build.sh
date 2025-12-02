#!/usr/bin/env bash
set -e

# Local-only image tag (same on PC and RP5)
IMAGE_NAME=${IMAGE_NAME:-elec:humble-local}
DOCKERFILE=${DOCKERFILE:-Dockerfile}
CONTEXT_DIR=${CONTEXT_DIR:-..}

echo "----------------------------------------"
echo " Building Docker image (local only) "
echo "----------------------------------------"
echo "Image:      $IMAGE_NAME"
echo "Dockerfile: $DOCKERFILE"
echo "Context:    $CONTEXT_DIR"
echo ""

docker build --progress=plain -t "$IMAGE_NAME" -f "$DOCKERFILE" "$CONTEXT_DIR"

echo ""
echo "--------------------------------------------------------------"
echo " Done. Image built locally as: $IMAGE_NAME"
echo " Architecture = this machine's arch (amd64 on PC, arm64 on RP5)"
echo "--------------------------------------------------------------"

