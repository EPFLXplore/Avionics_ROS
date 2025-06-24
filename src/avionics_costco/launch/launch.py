from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
import os

def generate_launch_description():
    ld = LaunchDescription()
    logger = LaunchConfiguration("log_level")

    ns = 'avionics_costco'

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    ))

    package_name = 'avionics_costco'
    executable_name = 'avionics_costco'

    # Launch the mux node with parameters
    Costco_node = Node(
        package=package_name,
        executable=executable_name,
        namespace=ns,
        output='screen',
        arguments=['--ros-args',
            '--log-level', logger,
            ]
    )
    ld.add_action(Costco_node)

    bms_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
            get_package_share_directory('avionics_BMS_new'), 'launch'
            ), '/avionics_bms_launch.py']
        )
    )

    ld.add_action(bms_launch)

    return ld
