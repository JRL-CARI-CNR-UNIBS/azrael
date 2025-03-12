from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name='robot_ip', default_value='192.168.254.31', description='ur net IP'),
    DeclareLaunchArgument(name='fake_ur', default_value='true', description='use fake hardware'),
    DeclareLaunchArgument(name='rviz', default_value='true', description='Load Rviz'),
    DeclareLaunchArgument(name='move_group', default_value='true', description='Start move group'),
    DeclareLaunchArgument(name='prefix', default_value='azrael', description='URDF prefix (without /)')
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):

  robot_description_path = PathJoinSubstitution([FindPackageShare('azrael_description'), 'urdf', 'system.urdf.xacro']).perform(context)
  robot_description_args = {
    'robot_ip' : LaunchConfiguration('robot_ip').perform(context),
    'fake_ur' : LaunchConfiguration('fake_ur').perform(context),
    'prefix' : f'{LaunchConfiguration("prefix").perform(context)}/',
  }

  srdf_path = PathJoinSubstitution([FindPackageShare('azrael_moveit_config'), 'config', 'azrael.srdf']).perform(context)
  joint_limits_path = PathJoinSubstitution([FindPackageShare('azrael_moveit_config'), 'config', 'joint_limits.yaml']).perform(context)
  moveit_controllers_path = PathJoinSubstitution([FindPackageShare('azrael_moveit_config'), 'config', 'moveit_controllers.yaml']).perform(context)
  rviz_config_path = PathJoinSubstitution([FindPackageShare('azrael_app'), 'rviz', 'setup.rviz'])

  moveit_config = (
    MoveItConfigsBuilder('azrael', package_name='azrael_moveit_config')
    .robot_description(file_path=robot_description_path, mappings=robot_description_args)
    .robot_description_semantic(file_path=srdf_path)
    .planning_scene_monitor(publish_robot_description=False,
                            publish_robot_description_semantic=True,
                            publish_planning_scene=True)
    .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
    .joint_limits(file_path=joint_limits_path)
    .trajectory_execution(file_path=moveit_controllers_path)
    .to_moveit_configs()
  )
  print("Robot Description Content:", moveit_config.robot_description)

  move_group_node = Node(
    package='moveit_ros_move_group',
    executable='move_group',
    output='screen',
    parameters=[moveit_config.to_dict()],
    condition=IfCondition(LaunchConfiguration('move_group'))
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(LaunchConfiguration('rviz')),
    parameters=[
      #moveit_config.to_dict()
    ],
    arguments=['-d', rviz_config_path],
  )

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='both',
    # condition=IfCondition(LaunchConfiguration('use_fake_hardware')),
    parameters=[moveit_config.robot_description]
  )

  return [
    move_group_node,
    rviz_node,
    robot_state_publisher_node,
  ]
