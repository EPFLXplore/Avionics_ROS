import os
from launch import LaunchDescription
from launch_ros.actions import Node

# This is the BMS launchfile
# It is what is called after ros2 launch

# This is the general form for a launch file -> look online if needed
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avionics_bms',
            executable='bms_monitor', # What will be run, it is bms_publisher in avioncs_bms/bms_monitor.py -> main
            name='bms_publisher',
            output='screen', # Allows to display logs in terminal
        )
    ])
