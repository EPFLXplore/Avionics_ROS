import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    logger = LaunchConfiguration("log_level")

    # CAN bus names
    # bus0_name = 'can0'
    # bus1_name = 'can1'
    
    # Namespace
    ns = 'avionics'

    ld.add_action(DeclareLaunchArgument(
        'bus',
        default_value='can0',
        description='CAN bus name'
    ))

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    ))

    config_package_name = "avionics_config"
    config_executable_name = "avionics_config"

    topic_names_params_file = os.path.join(
        get_package_share_directory(config_package_name),
        'config',
        'el_interface_names.yaml'
    )

    # # Node IDs and ping parameters config file
    # id_params_file = os.path.join(
    #     get_package_share_directory(config_package_name),
    #     'config',
    #     'node_ids_params.yaml'
    # )

    # # Node IDs and ping parameters config file
    # connection_params_file = os.path.join(
    #     get_package_share_directory(config_package_name),
    #     'config',
    #     'connection_params.yaml'
    # )


    # # Calibration config file
    # calibration_params_file = os.path.join(
    #     get_package_share_directory(config_package_name),
    #     'config',
    #     'calibration_params.yaml'
    # )

    # Launch the mux node with parameters
    # node_config = Node(
    #     package=config_package_name,
    #     executable=config_executable_name,
    #     namespace=ns,
    #     parameters=[topic_names_params_file, id_params_file, calibration_params_file, connection_params_file],
    #     output='screen',
    #     arguments=['--ros-args',
    #         '--log-level', logger]
    # )
    # ld.add_action(node_config)

    # # Define the package and executable names
    package_name = 'avionics_mux'
    executable_name = 'avionics_mux'

    # Iterate through the namespaces
    # for bus in [bus0_name, bus1_name]:

    # Launch the node for the namespace with parameters
    can_node = Node(
        package=package_name,
        executable=executable_name,
        namespace=ns,# + "/" + bus,
        parameters=[topic_names_params_file] #,id_params_file, calibration_params_file, connection_params_file],
        output='screen',
        arguments=['--ros-args',
            '--log-level', logger,
            '--param', 'bus:=' + bus,
            '--param', 'topic_prefix:=' + bus]
    )
    ld.add_action(can_node)

    # # Define the package and executable names for the mux node
    # mux_package_name = 'avionics_mux'
    # mux_executable_name = 'avionics_mux'

    # # Launch the mux node with parameters
    # node_mux = Node(
    #     package=mux_package_name,
    #     executable=mux_executable_name,
    #     namespace=ns,
    #     parameters=[topic_names_params_file, id_params_file, calibration_params_file, connection_params_file],
    #     output='screen',
    #     arguments=['--ros-args',
    #         '--log-level', logger,
    #         '--param', 'bus0:=' + bus0_name,
    #         '--param', 'bus1:=' + bus1_name]
    # )
    # ld.add_action(node_mux)

    # # Define the package and executable names for the converter node
    # converter_package_name = 'avionics_converter'
    # converter_executable_name = 'avionics_converter'

    # # Launch the mux node with parameters
    # node_converter = Node(
    #     package=converter_package_name,
    #     executable=converter_executable_name,
    #     namespace=ns,
    #     parameters=[topic_names_params_file, id_params_file, calibration_params_file, connection_params_file],
    #     output='screen',
    #     arguments=['--ros-args',
    #         '--log-level', logger]
    # )
    # ld.add_action(node_converter)

    return ld
