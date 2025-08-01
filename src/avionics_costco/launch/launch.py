# Author: Eliot Abramo, Matas Jones

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from typing import List, Tuple


def declare_launch_argument(name: str, default_value: str, choices: List[str] = None, description: str = "") -> Tuple[LaunchConfiguration, DeclareLaunchArgument]:
    # Initialize the LaunchConfiguration
    arg = LaunchConfiguration(name)

    # Declare the launch argument with a default value
    declare_arg = DeclareLaunchArgument(
        name,
        default_value=default_value,
        description=description,
        choices=choices
    )
    
    return arg, declare_arg

def generate_launch_description():
    
    ns = 'avionics_costco'
    package_name = 'avionics_costco'
    executable_name = 'avionics_costco'
    
    logger_arg, logger_declaration = declare_launch_argument("log_level", default_value="info", description="Logger level")
    
    Costco_node_avionics = Node(
        package=package_name,
        executable=executable_name,
        namespace=ns,
        output='screen',
        arguments=['--ros-args',
            '--log-level', logger_arg,
            ],
        parameters=[
            {"port_name": '/dev/ttyESP32_Avionics'},
        ]
    )

    python_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
            get_package_share_directory('python_node'), 'launch'
            ), '/python_launch.py']
        )
    )

    # Declare all the steps of the launch file process
    return LaunchDescription([
        logger_declaration,
        Costco_node_avionics,
        python_launch
        ]
    )