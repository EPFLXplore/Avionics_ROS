#!/usr/bin/env bash
set -e

echo "Closing service module: servo 1 -> 0°, servo 2 -> 180°"

ros2 topic pub --once /servo_angle custom_msg/msg/ServoRequest \
    "{id: 1, increment: 0, zero_in: true}"

ros2 topic pub --once /servo_angle custom_msg/msg/ServoRequest \
    "{id: 2, increment: 180, zero_in: false}"

echo "Done."
