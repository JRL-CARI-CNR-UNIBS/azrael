from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([OpaqueFunction(function=launch_setup)])

def launch_setup(context):

  return [
    IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource(
        launch_file_path=PathJoinSubstitution([FindPackageShare('azrael_app'), 'launch', 'moveit.launch.py'])
      )
    ),
    IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource(
        launch_file_path=PathJoinSubstitution([FindPackageShare('azrael_app'), 'launch', 'controllers.launch.py'])
      )
    )
  ]
