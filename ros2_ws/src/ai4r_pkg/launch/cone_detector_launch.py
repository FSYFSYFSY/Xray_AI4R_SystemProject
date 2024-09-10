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
    
    # # Construct the path to the YAML config file
    # config = os.path.join(
    #     get_package_share_directory('ai4r_pkg'),
    #     'config',
    #     'config_for_line_detector.yaml'
    # )

    # # Convert the YAML config file to a Python dictionary
    # with open(config) as yaml_file:
    #     # use safe_load instead load
    #     config_yaml_as_dict = yaml.safe_load(yaml_file)

    # # Extract the element of the dictionary that needs to be passed to the node
    # parameters_dict_for_node = config_yaml_as_dict["line_detector_node"]["ros__parameters"]

    # Print the dictionary for debugging
    #print(parameters_dict_for_node)

    # Node launch details
    node=Node(
        package='ai4r_pkg',
        # namespace=LaunchConfiguration('agent_ns'),
        executable='cone_detector_node.py',
        name='cone_detector_node',
        # parameters = [parameters_dict_for_node]
    )

    # ld.add_action(agent_ns_launch_arg)
    ld.add_action(node)
    return ld