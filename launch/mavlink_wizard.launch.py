#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for MAVLink Wizard"""

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for MAVLink communication'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    auto_scan_arg = DeclareLaunchArgument(
        'auto_scan',
        default_value='true',
        description='Automatically scan for devices on startup'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Configuration file to load on startup'
    )

    # Get package directories
    mavlink_wizard_dir = get_package_share_directory('mavlink_wizard')
    stm32_mavlink_dir = get_package_share_directory('stm32_mavlink_interface')

    # Configuration file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('mavlink_wizard'),
        'config',
        'wizard_config.yaml'
    ])

    # STM32 MAVLink Interface Node
    stm32_mavlink_node = Node(
        package='stm32_mavlink_interface',
        executable='mavlink_serial_node',
        name='mavlink_serial_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baud_rate'),  # Note: parameter name is 'baudrate' not 'baud_rate'
            'system_id': 1,
            'component_id': 1,
            'target_system_id': 1,
            'target_component_id': 1,
        }],
        output='screen'
    )

    # MAVLink Wizard GUI Node
    mavlink_wizard_gui_node = ExecuteProcess(
        cmd=['python3', PathJoinSubstitution([
            FindPackagePrefix('mavlink_wizard'),
            'lib',
            'mavlink_wizard',
            'mavlink_wizard_gui.py'
        ])],
        name='mavlink_wizard_gui',
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        auto_scan_arg,
        config_file_arg,
        stm32_mavlink_node,
        mavlink_wizard_gui_node,
    ])