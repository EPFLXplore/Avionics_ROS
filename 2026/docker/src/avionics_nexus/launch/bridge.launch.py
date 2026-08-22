"""Bring up the avionics bridge.

Detection now lives in C++: the single avionics_nexus process always constructs
one Nexus node per master id 0..3 (namespaces /ttyNova0../ttyNova3). Masters that
are plugged in work immediately; absent ones sit in their reconnect loop, warning
periodically, and attach the moment they appear (hotplug, no relaunch).

Alongside it, qos_bridge republishes every command topic as <topic>_latched with
TRANSIENT_LOCAL durability, and avionics_nexus is pointed at those. That is what
lets a Nexus which matches LATE - a restart, a redeploy, or the port reconnect its
RX thread does on every unplug - still receive the last command instead of never
seeing it. Commands are one-shots that nothing re-sends, so a miss in that window
is a failed operator action with no trace.

    ros2 launch avionics_nexus bridge.launch.py                        # bridged
    ros2 launch avionics_nexus bridge.launch.py use_qos_bridge:=false  # raw topics

Both flip together on purpose: the suffix and the bridge are the same decision,
and setting one without the other means avionics_nexus subscribes to a topic
nobody publishes.

It does NOT rescue `ros2 topic pub --once`. That message is lost because the
PUBLISHER exits before discovery completes, so it reaches no subscriber at all -
the bridge included, which then has nothing to latch. Pass `-w 1`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    use_bridge = LaunchConfiguration("use_qos_bridge")

    # The suffix is derived from the same flag rather than being its own argument,
    # so the two can never disagree.
    command_suffix = PythonExpression(
        ["'_latched' if '", use_bridge, "'.lower() == 'true' else ''"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_qos_bridge",
            default_value="true",
            description="Route commands through qos_bridge (<topic>_latched, "
                        "TRANSIENT_LOCAL) so a late-joining Nexus still gets the "
                        "last command. false = subscribe to the raw topics.",
        ),

        Node(
            package="avionics_nexus",
            executable="qos_bridge",
            name="qos_bridge",
            output="screen",
            condition=IfCondition(use_bridge),
        ),

        Node(
            package="avionics_nexus",
            executable="avionics_nexus",
            name="avionics_nexus",
            output="screen",
            parameters=[config_file("mass_cal.yaml"), config_file("ph_cal.yaml"),
                        config_file("servo_cal.yaml"),
                        {"command_suffix": command_suffix}],
        ),
    ])
