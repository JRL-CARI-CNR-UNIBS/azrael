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

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="azrael_description",
            description="mobile manip description",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="system.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            # " ","safety_limits:=",safety_limits,
            # " ","safety_pos_margin:=",safety_pos_margin,
            # " ","safety_k_position:=",safety_k_position,
            " ","name:=","azrael",
            " ","ur_type:=","ur10",
            " ","prefix:=","azrael/",
            # " ","prefix_rc:=","azrael",
            # " ","simulation_controllers:=",initial_joint_controllers_1,
            # " ","use_fake_hardware:=",use_fake_hardware,
            # " ","sim_gazebo:=",sim_gazebo,
        ]
    )



    azrael_driver_udp = Node(
        package="azrael_driver_udp",
        namespace='azrael',
        executable="azrael_driver_udp_node",
        output="log")

    rplidar =   Node(
            name='sllidar_ros2',
            package='sllidar_ros2',
            namespace='azrael',
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

    robot_description_1  = {"robot_description": robot_description_content}
    frame_prefix_param_1 = {"frame_prefix": ""}


    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace='azrael',
        output="log",
        parameters=[robot_description_1,frame_prefix_param_1])

    nodes_to_start = [
        rplidar,
        azrael_driver_udp,
        robot_state_publisher_node_1
    ]

    return LaunchDescription(nodes_to_start)

