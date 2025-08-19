import os
from launch import LaunchDescription
from launch_ros.actions import Node

# This is the python node launchfile
# It is what is called after ros2 launch

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_node',
            executable='python_node',
            name='python_node',
            output='screen', # Allows to display logs in terminal
        )
    ])