from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
  launch_args = [
    #DeclareLaunchArgument(name='azrael_ip', default_value=None, description='Azrael net IP'),
    #DeclareLaunchArgument(name='azrael_username', default_value=None, description='Azrael local username'),
    DeclareLaunchArgument(name='ur_local_ip', default_value='192.168.254.31', description='ur local net IP'),
    DeclareLaunchArgument(name='use_fake_hardware', default_value='false', description='use fake hardware'),
    DeclareLaunchArgument(name='rviz', default_value='true', description='Load Rviz'),
    DeclareLaunchArgument(name='move_group', default_value='false', description='Start move group'),
    DeclareLaunchArgument(name='prefix', default_value='azrael', description='ROS Namespace')
  ]

  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):

  #remote_launch_command = "bash -i -c 'ros2 launch azrael_app on_robot.launch.py'"

  return [
  GroupAction(
    actions=[
      PushRosNamespace(LaunchConfiguration('prefix')),
      # IMM Controller
#      IncludeLaunchDescription(
#        launch_description_source=PythonLaunchDescriptionSource(
#          launch_file_path=PathJoinSubstitution([FindPackageShare('azrael_app'), 'launch', 'spawn_controllers.launch.py'])
#        )
#      ),
      # Cartesian Controller
      IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
          launch_file_path=PathJoinSubstitution([FindPackageShare('azrael_app'), 'launch', 'spawn_cartesian_controller.launch.py'])
        )
      ),
      Node(
        package='rviz2',
        executable='rviz2',
        parameters=[
          #moveit_config.to_dict()
        ],
        arguments=['-d', PathJoinSubstitution([FindPackageShare('azrael_app'), 'rviz', 'setup.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
      )
      # ExecuteProcess(
      #   cmd=['ssh', '-t', f'{LaunchConfiguration("azrael_username").perform(context)}@{LaunchConfiguration("azrael_ip").perform(context)}',
      #     f'{remote_launch_command}'],
      #   output='both'
      # )
    ]
  )
  ]
