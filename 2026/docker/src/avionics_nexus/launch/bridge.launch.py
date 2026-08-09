"""Bring up the avionics bridge.

Detection now lives in C++: the single avionics_nexus process always constructs
one Nexus node per master id 0..3 (namespaces /ttyNova0../ttyNova3). Masters that
are plugged in work immediately; absent ones sit in their reconnect loop, warning
periodically, and attach the moment they appear (hotplug, no relaunch). So this
launch file just starts that one process.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def config_file(name):
    """Path to the load-cell calibration.

    Prefer the copy in the bind-mounted src/ tree (run.sh mounts the host's src/
    at ~/dev_ws/src): editing a slope there takes effect on the next node start,
    with no colcon build to re-copy it into install/. Outside the dev container
    that path is absent, so fall back to the installed share/ copy.
    """
    src = os.path.expanduser("~/dev_ws/src/avionics_nexus/config/" + name)
    if os.path.exists(src):
        return src
    return os.path.join(get_package_share_directory("avionics_nexus"), "config", name)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="avionics_nexus",
            executable="avionics_nexus",
            name="avionics_nexus",
            output="screen",
            parameters=[config_file("mass_cal.yaml"), config_file("ph_cal.yaml"),
                        config_file("servo_cal.yaml")],
        )
    ])
