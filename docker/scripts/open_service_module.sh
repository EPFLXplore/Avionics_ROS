#!/usr/bin/env bash
set -e

echo "Opening service module: servo 1 -> 180°, servo 2 -> 0°"

ros2 topic pub --once /servo_angle custom_msg/msg/ServoRequest \
    "{id: 1, increment: 180, zero_in: false}"

ros2 topic pub --once /servo_angle custom_msg/msg/ServoRequest \
    "{id: 2, increment: 0, zero_in: true}"

echo "Done."
