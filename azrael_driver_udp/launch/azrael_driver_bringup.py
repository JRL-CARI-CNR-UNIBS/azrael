#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import LogInfo
from launch_ros.actions import Node



def generate_launch_description():




    azrael_driver_udp = Node(
        package="azrael_driver_udp",
        namespace='azrael',
        executable="azrael_driver_udp_node",
        output="log")

    # rplidar = Node(
    #     package='rplidar_ros2',
    #     executable='rplidar_scan_publisher',
    #     namespace='azrael',
    #     name='rplidar_scan_publisher',
    #     parameters=[{'serial_port': "/dev/ttyUSB0", 
    #                     'serial_baudrate': 256000, 
    #                     'frame_id': "azrael/laser",
    #                     'inverted': False, 
    #                     'angle_compensate': True, 
    #                     'channel_type': "",
    #                     'tcp_ip': "1.1.1.1",
    #                     'udp_ip': "1.1.1.1",
    #                     'tcp_port': 11,
    #                     'udp_port': 11,
    #                     'scan_frequency': 10.0,
    #                     'scan_mode': "Sensitivity"}],

    #     output='log')

    rplidar =   Node(
            name='sllidar_ros2',
            package='sllidar_ros2',
            executable='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,  # A3
                'frame_id': 'azrael/laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
        )


    nodes_to_start = [
        rplidar,
        azrael_driver_udp,
    ]

    return LaunchDescription(nodes_to_start)

