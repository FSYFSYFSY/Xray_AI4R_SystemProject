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

    # Node launch details
    node=Node(
        package='ai4r_pkg',
        # namespace=LaunchConfiguration('agent_ns'),
        executable='stream_oak_node.py',
        name='stream_oak_node',
    )

    # ld.add_action(agent_ns_launch_arg)
    ld.add_action(node)
    return ld