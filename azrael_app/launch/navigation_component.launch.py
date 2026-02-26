from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description() -> LaunchDescription:

  args = [
    DeclareLaunchArgument(name='namespace', 
                          default_value='azrael', 
                          description='namespace of each node'),
    DeclareLaunchArgument(name='autostart', 
                          default_value='true', 
                          description='autostart nav nodes'),
    DeclareLaunchArgument(name='use_composition', 
                        default_value='true', 
                        description='use composite nav nodes'),
    DeclareLaunchArgument(name='container_name',
                          default_value='azrael_nav_container',
                          description='Container name if using composition'),
  ]

  namespace = LaunchConfiguration('namespace')
  use_composition = LaunchConfiguration('use_composition')
  container_name = LaunchConfiguration('container_name')

  lifecycle_nodes = [ f'/{namespace}/{node}' for node in
    [
      'bt_navigator',
      'behavior_server',
      'waypoint_follower',
      'planner_server',
      'controller_server',
      'smoother_server',
    ]
  ]

  nav_params_azrael = PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'nav_params.yaml'])

  independent_nodes = GroupAction(
    condition=IfCondition(NotSubstitution(use_composition)),
    actions=[
      PushRosNamespace(namespace=namespace),
      Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        parameters=[nav_params_azrael],
      ),

      Node(
        package='nav2_behaviors',
        executable='behavior_server',
        parameters=[nav_params_azrael],

      ),

      Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        parameters=[nav_params_azrael],

      ),

      Node(
        package='nav2_planner',
        executable='planner_server',
        parameters=[nav_params_azrael],

      ),

      Node(
        package='nav2_controller',
        executable='controller_server',
        parameters=[nav_params_azrael],
      ),

      Node(
        package='nav2_smoother',
        executable='smoother_server',
        parameters=[nav_params_azrael],
      ),

      Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': LaunchConfiguration('autostart')}, 
                    {'node_names': lifecycle_nodes}, 
                    {'bond_timeout': 0.0}],
      ),
  ])

  composite_nodes = GroupAction(
    condition=IfCondition(use_composition),
    actions=[
      LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
          ComposableNode(
            package='nav2_bt_navigator',
            plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator',
            namespace=namespace,
            parameters=[nav_params_azrael],
          ),

          ComposableNode(
            package='nav2_behaviors',
            plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            namespace=namespace,
            parameters=[nav_params_azrael],
          ),

          ComposableNode(
            package='nav2_waypoint_follower',
            plugin='nav2_waypoint_follower::WaypointFollower',
            name='waypoint_follower',
            namespace=namespace,
            parameters=[nav_params_azrael],
          ),

          ComposableNode(
            package='nav2_planner',
            plugin='nav2_planner::PlannerServer',
            name='planner_server',
            namespace=namespace,
            parameters=[nav_params_azrael],
          ),

          ComposableNode(
            package='nav2_controller',
            plugin='nav2_controller::ControllerServer',
            name='controller_server',
            namespace=namespace,
            parameters=[nav_params_azrael],
          ),

          ComposableNode(
            package='nav2_smoother',
            plugin='nav2_smoother::SmootherServer',
            name='smoother_server',
            namespace=namespace,
            parameters=[nav_params_azrael],
          ),

          ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            parameters=[{'autostart': LaunchConfiguration('autostart')}, 
                        {'node_names': lifecycle_nodes}, 
                        {'bond_timeout': 0.0}],
          ),
        ]
      )
    ]
  )

  return LaunchDescription([
    *args,
    independent_nodes,
    composite_nodes
  ])
