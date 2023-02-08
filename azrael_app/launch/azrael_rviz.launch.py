import os
from os import environ
from os import pathsep
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import OpaqueFunction
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths


def generate_launch_description():


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("azrael_app"), "rviz", "config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )



    nodes_to_start = [
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)


