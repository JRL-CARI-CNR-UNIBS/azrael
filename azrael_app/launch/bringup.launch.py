from launch.conditions import UnlessCondition
from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name='robot_ip', default_value='192.168.254.31', description='ur net IP'),
    DeclareLaunchArgument(name='use_fake_hardware', default_value='false', description='use fake hardware'),
    DeclareLaunchArgument(name='rviz', default_value='true', description='Load Rviz'),
    DeclareLaunchArgument(name='move_group', default_value='false', description='Start move group'),
    DeclareLaunchArgument(name='prefix', default_value='azrael', description='ROS Namespace')
  ]

  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):

  return [
  GroupAction(
    actions=[
      PushRosNamespace(LaunchConfiguration('prefix')),
      IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
          launch_file_path=PathJoinSubstitution([FindPackageShare('azrael_app'), 'launch', 'moveit.launch.py'])
        ),
        launch_arguments=[
          ('robot_ip', LaunchConfiguration('robot_ip')),
          ('rviz', LaunchConfiguration('rviz')),
          ('move_group', LaunchConfiguration('move_group')),
          ('use_fake_hardware', LaunchConfiguration('use_fake_hardware')),
          ('prefix', LaunchConfiguration('prefix')),
        ]
      ),
      IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
          launch_file_path=PathJoinSubstitution([FindPackageShare('azrael_app'), 'launch', 'controllers.launch.py'])
        )
      ),
      IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
          launch_file_path=PathJoinSubstitution([FindPackageShare('azrael_app'), 'launch', 'mobile_base.launch.py'])
        ),
        condition=UnlessCondition(LaunchConfiguration('use_fake_hardware'))
      ),
    ]
  )
  ]
