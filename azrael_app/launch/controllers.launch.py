from launch.launch_description import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name='prefix', default_value='azrael', description='Controller Manager prefix')
  ]

  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):

  ros2_control_config_path = PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'ros2_controllers.yaml'])

  controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[ros2_control_config_path],
    # prefix='gnome-terminal -- cgdb -ex run --args',
    output='screen',
    remappings=[('controller_manager/robot_description','robot_description')],
  )

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
      '--controller-manager', f'{LaunchConfiguration("prefix").perform(context)}/controller_manager'],
  )

  # https://github.com/muttistefano/imm_controller.git
  imm_controller = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['imm_controller',
      '--controller-manager', f'{LaunchConfiguration("prefix").perform(context)}/controller_manager'],
  )

  return [
    controller_manager_node,
    TimerAction(
      actions=[joint_state_broadcaster_spawner,
              #joint_trajectory_controller_spawner,
              imm_controller],
      period=3.0,
    )
  ]
