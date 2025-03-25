from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()
    logger = LaunchConfiguration("log_level")

    ns = 'CostcoPublisher'

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    ))

    package_name = 'CostcoPublisher'
    executable_name = 'CostcoPublisher'

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

    return ld