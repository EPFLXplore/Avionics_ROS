#!/usr/bin/env bash
set -e

echo "Closing service module"

ros2 topic pub --once /servo_angle custom_msg/msg/ServoRequest \
    "{id: 10, increment: 180, zero_in: false}"

echo "Done."
