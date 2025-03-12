from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name='robot_ip', default_value='192.168.254.31', description='ur net IP'),
    DeclareLaunchArgument(name='fake_ur', default_value='false', description='use fake hardware'),
    DeclareLaunchArgument(name='rviz', default_value='true', description='Load Rviz'),
    DeclareLaunchArgument(name='move_group', default_value='false', description='Start move group'),
    DeclareLaunchArgument(name='prefix', default_value='azrael', description='URDF prefix (without /)')
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
  rviz_config_path = PathJoinSubstitution([FindPackageShare('azrael_app'), 'rviz', 'setup.rviz'])

  robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('azrael_description'), "urdf", 'system.urdf.xacro']).perform(context),
            
        ]
    )
  
  state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'robot_description': robot_description_content}],
      # namespace=ns_str,
  )
  joint_state_publisher_gui = LaunchConfiguration('js_publisher_gui', default=True)

  joint_state_publisher_node = Node(
          package='joint_state_publisher_gui',
          executable='joint_state_publisher_gui',
          name='joint_state_publisher_gui',
          # namespace=ns_str,
          condition=IfCondition(joint_state_publisher_gui)
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

  return [
    state_publisher_node,
    rviz_node,
    joint_state_publisher_node
  ]
