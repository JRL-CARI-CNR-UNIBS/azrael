from launch.launch_description import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  launch_args = [
    #DeclareLaunchArgument(name='prefix', default_value='azrael', description='Controller Manager prefix')
  ]

  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):

  # Parameters are set by the spawner instead of being loaded by the controller manager!

  ros2_control_config_path = PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'ros2_controllers.yaml'])

  # joint_trajectory_controller_spawner = Node(
  #   package='controller_manager',
  #   executable='spawner',
  #   arguments=['joint_trajectory_controller',
  #              '--controller-manager', 'controller_manager'],
  # )

  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster',
      '--controller-manager', 'controller_manager',
      '--controller-manager-timeout', '10',
      '--param-file', ros2_control_config_path],
  )

  # https://github.com/muttistefano/imm_controller.git
  cartesian_motion_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['cartesian_motion_controller',
      '--controller-manager', 'controller_manager',
      '--controller-manager-timeout', '10',
      '--param-file', ros2_control_config_path],
  )

  return [
      joint_state_broadcaster_spawner,
      cartesian_motion_controller_spawner,
  ]
