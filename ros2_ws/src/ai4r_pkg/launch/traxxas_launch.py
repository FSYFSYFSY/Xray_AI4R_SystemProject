import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Initialise the Launch Description variable
    ld = LaunchDescription()

    # # Declare input arguments that this launch file accepts
    # agent_ns_launch_arg = DeclareLaunchArgument(
    #     'agent_ns',
    #     default_value='agent01'
    # )

    config = os.path.join(
        get_package_share_directory('ai4r_pkg'),
        'config',
        'traxxas_node_params.yaml'
        )
    
    # Convert the YAML config file to a Python dictionary
    with open(config) as yaml_file:
        # use safe_load instead load
        config_yaml_as_dict = yaml.safe_load(yaml_file)

    # Extract the element of the dictionary that needs to be passed to the node
    parameters_dict_for_node = config_yaml_as_dict["traxxas_node"]["ros__parameters"]

    # Node launch details
    traxxas_node=Node(
        package='ai4r_pkg',
        # namespace=LaunchConfiguration('agent_ns'),
        executable='traxxas',
        name='traxxas_node',
        parameters=[parameters_dict_for_node]
    )
    rc_switch_node=Node(
        package='ai4r_pkg',
        # namespace=LaunchConfiguration('agent_ns'),
        executable='rc_switch_node',
        name='rc_switch_node'
    )


    # ld.add_action(agent_ns_launch_arg)
    ld.add_action(traxxas_node)
    ld.add_action(rc_switch_node)
    return ld