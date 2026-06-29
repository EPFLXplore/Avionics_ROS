#!/usr/bin/env bash
set -e

echo "Changing leds"

ros2 topic pub --once /EL/LedCommands custom_msg/msg/LEDRequest \
    "{system: 0, mode: 1}"
    
ros2 topic pub --once /EL/LedCommands custom_msg/msg/LEDRequest \
    "{system: 1, mode: 1}"

ros2 topic pub --once /EL/LedCommands custom_msg/msg/LEDRequest \
    "{system: 2, mode: 1}"
    
ros2 topic pub --once /EL/LedCommands custom_msg/msg/LEDRequest \
    "{system: 3, mode: 1}"

echo "Waiting 10s..."
sleep 10

ros2 topic pub --once /EL/LedCommands custom_msg/msg/LEDRequest \
    "{system: 2, mode: 2}"

echo "Done."
