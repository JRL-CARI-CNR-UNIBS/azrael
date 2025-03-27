from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, Node

import os


def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name='launch_ur', default_value='true', description='Launch ur robot launcher'),
    DeclareLaunchArgument(name='robot_ip', default_value='192.168.254.100', description='ur local net IP'),
  ]

  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):
    azrael_dir = get_package_share_directory('azrael_app')
    launch_dir = os.path.join(azrael_dir, 'launch')
    on_robot_dir = os.path.join(azrael_dir, 'launch/on_robot')

from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, Node

import os


def generate_launch_description():
    azrael_dir = get_package_share_directory('azrael_app')
    launch_dir = os.path.join(azrael_dir, 'launch')
    on_robot_dir = os.path.join(azrael_dir, 'launch/on_robot')

    robot_ip = LaunchConfiguration('robot_ip')
    launch_ur = LaunchConfiguration('launch_ur')

    launch_ur_cmd = DeclareLaunchArgument(name='launch_ur', default_value='true', description='Launch ur robot launcher')
    robot_ip_cmd = DeclareLaunchArgument(name='robot_ip', default_value='192.168.254.100', description='ur local net IP')

    bringup_cmd_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'ur_bringup.launch.py')
                ),
                condition=IfCondition(launch_ur),
                launch_arguments={
                    'launch_rviz': "false",
                    'fake_ur': "false",
                    'headless_mode': "true",
                    'robot_ip': robot_ip
                }.items(),
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(on_robot_dir, 'on_robot.launch.py')
            #     ),
            # ),
        ]
    )
    ld = LaunchDescription()
    ld.add_action(launch_ur_cmd)
    ld.add_action(robot_ip_cmd)
    ld.add_action(bringup_cmd_group)
    
    return ld