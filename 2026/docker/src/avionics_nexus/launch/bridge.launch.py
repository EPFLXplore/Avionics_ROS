"""Bring up the avionics bridge.

Detection now lives in C++: the single avionics_nexus process always constructs
one Nexus node per master id 0..3 (namespaces /ttyNova0../ttyNova3). Masters that
are plugged in work immediately; absent ones sit in their reconnect loop, warning
periodically, and attach the moment they appear (hotplug, no relaunch). So this
launch file just starts that one process.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="avionics_nexus",
            executable="avionics_nexus",
            name="avionics_nexus",
            output="screen",
        )
    ])
