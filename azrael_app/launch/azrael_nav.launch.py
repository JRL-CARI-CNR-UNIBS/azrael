from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  args = [
    DeclareLaunchArgument(name='ns', default_value='azrael', description='namespace of each node'),
    DeclareLaunchArgument(name='autostart', default_value='true', description='autostart nav nodes')

  ]

  return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])

def launch_setup(context):
  nav_params_azrael = PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'azrael_nav.yaml'])

#  azrael has its own localization system
  azrael_amcl = Node(
   package='nav2_amcl',
   executable='amcl',
   parameters=[nav_params_azrael],
  )

  azrael_bt_navigator = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    parameters=[nav_params_azrael],
    # arguments=['--ros-args', '--log-level', 'debug']
  )

  azrael_behavior_server = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    parameters=[nav_params_azrael],
    # arguments=['--ros-args', '--log-level', 'debug']

  )

  azrael_waypoint_follower = Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    parameters=[nav_params_azrael],
    # arguments=['--ros-args', '--log-level', 'debug']

  )

  azrael_planner_server = Node(
    package='nav2_planner',
    executable='planner_server',
    parameters=[nav_params_azrael],
    # arguments=['--ros-args', '--log-level', 'debug']

  )

  azrael_controller_server = Node(
    package='nav2_controller',
    executable='controller_server',
    parameters=[nav_params_azrael],
    # arguments=['--ros-args', '--log-level', 'debug']

  )

  azrael_smoother_server = Node(
    package='nav2_smoother',
    executable='smoother_server',
    parameters=[nav_params_azrael],
    # arguments=['--ros-args', '--log-level', 'debug']

  )

  #static_trasform_publisher
#   map_static_trasform = Node(
#     package='tf2_ros',
#     executable='static_transform_publisher',
#     arguments=['--frame-id', 'map',
#                '--child-frame-id', 'azrael/odom']
#   )
  
  lifecycle_nodes = [
    '/azrael/behavior_server',
    '/azrael/bt_navigator',
    '/azrael/controller_server',
    '/azrael/planner_server',
    '/azrael/smoother_server',
    '/azrael/waypoint_follower',
  ]

  azrael_nav2_lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    output='screen',
    parameters=[{'autostart': LaunchConfiguration('autostart')}, 
                {'node_names': lifecycle_nodes}, 
                {'bond_timeout': 0.0}],
  )

#   point_to_laser_cmd = Node(
#         package='pointcloud_to_laserscan', 
#         executable='pointcloud_to_laserscan_node',
#         # remappings=[('cloud_in', f'/azrael/cloud_in')], # topic per laser: scan 
#         parameters=[pointcloud_to_laserscan_param],
#         name='pointcloud_to_laserscan'
#     )

  

  azrael_nav_group = GroupAction(
    actions=[
            #  azrael_amcl,
             PushRosNamespace(LaunchConfiguration('ns')),
             azrael_bt_navigator,
             azrael_planner_server,
             azrael_controller_server,
             azrael_smoother_server,
             azrael_behavior_server,
             azrael_waypoint_follower,
             azrael_nav2_lifecycle_manager_node,
            #  map_static_trasform,
            #  point_to_laser_cmd
             ]
  )

  return [azrael_nav_group]