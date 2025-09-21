from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    i2c_device_arg = DeclareLaunchArgument(
        'i2c_device',
        default_value='/dev/i2c-1',
        description='I2C device path exposed by the host (e.g. /dev/i2c-1).'
    )

    sensor_address_arg = DeclareLaunchArgument(
        'sensor_address',
        default_value='41',
        description='Decimal I2C address of the VL53L5CX sensor (default 0x29 = 41).'
    )

    sensor_id_arg = DeclareLaunchArgument(
        'sensor_id',
        default_value='0',
        description='Identifier to embed in the published ai4r_interfaces/TofSensor messages.'
    )

    poll_interval_arg = DeclareLaunchArgument(
        'poll_interval_ms',
        default_value='50',
        description='Wall-timer polling period for the sensor driver in milliseconds.'
    )

    ranging_frequency_arg = DeclareLaunchArgument(
        'ranging_frequency_hz',
        default_value='20',
        description='Internal ranging frequency requested from the VL53L5CX sensor.'
    )

    integration_time_arg = DeclareLaunchArgument(
        'integration_time_ms',
        default_value='50',
        description='Integration time in milliseconds (only used in autonomous mode).'
    )

    sharpener_percent_arg = DeclareLaunchArgument(
        'sharpener_percent',
        default_value='20',
        description='Digital sharpener percentage applied by the VL53L5CX firmware.'
    )

    target_order_arg = DeclareLaunchArgument(
        'target_order',
        default_value='1',
        description='Target ordering strategy (1=closest, 2=strongest).'
    )

    ranging_mode_arg = DeclareLaunchArgument(
        'ranging_mode',
        default_value='1',
        description='Ranging mode (1=continuous, 3=autonomous).'
    )

    vl53l5cx_node = Node(
        package='ai4r_pkg',
        executable='vl53l5cx_node',
        name='vl53l5cx_node',
        parameters=[{
            'i2c_device': LaunchConfiguration('i2c_device'),
            'sensor_address': ParameterValue(LaunchConfiguration('sensor_address'), value_type=int),
            'sensor_id': ParameterValue(LaunchConfiguration('sensor_id'), value_type=int),
            'poll_interval_ms': ParameterValue(LaunchConfiguration('poll_interval_ms'), value_type=int),
            'ranging_frequency_hz': ParameterValue(LaunchConfiguration('ranging_frequency_hz'), value_type=int),
            'integration_time_ms': ParameterValue(LaunchConfiguration('integration_time_ms'), value_type=int),
            'sharpener_percent': ParameterValue(LaunchConfiguration('sharpener_percent'), value_type=int),
            'target_order': ParameterValue(LaunchConfiguration('target_order'), value_type=int),
            'ranging_mode': ParameterValue(LaunchConfiguration('ranging_mode'), value_type=int),
        }]
    )

    ld.add_action(i2c_device_arg)
    ld.add_action(sensor_address_arg)
    ld.add_action(sensor_id_arg)
    ld.add_action(poll_interval_arg)
    ld.add_action(ranging_frequency_arg)
    ld.add_action(integration_time_arg)
    ld.add_action(sharpener_percent_arg)
    ld.add_action(target_order_arg)
    ld.add_action(ranging_mode_arg)
    ld.add_action(vl53l5cx_node)

    return ld

