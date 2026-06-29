#!/usr/bin/env bash
set -e

echo "Opening service module"

ros2 topic pub --once /servo_angle custom_msg/msg/ServoRequest \
    "{id: 10, increment: 0, zero_in: false}"

echo "Done."
