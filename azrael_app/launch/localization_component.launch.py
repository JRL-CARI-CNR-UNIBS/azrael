from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotSubstitution, EqualsSubstitution, NotEqualsSubstitution, AndSubstitution
from launch.conditions import IfCondition, UnlessCondition

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
    DeclareLaunchArgument(name='load_map_server',
                          default_value='true',
                          description='Load map server node'),
    DeclareLaunchArgument(name='map',
                          default_value='',
                          description='Full path to map yaml file to load'),
  ]

  autostart = LaunchConfiguration('autostart')
  namespace = LaunchConfiguration('namespace')
  use_composition = LaunchConfiguration('use_composition')
  container_name = LaunchConfiguration('container_name')
  map_yaml_file = LaunchConfiguration('map')
  load_map_server = LaunchConfiguration('load_map_server')



  lifecycle_nodes = [ f'/{namespace}/{node}' for node in
    [
      'amcl'
    ]
  ]

  lifecycle_nodes_w_map_server = lifecycle_nodes + [ f'/{namespace}/{node}' for node in
    [
      'map_server'
    ]
  ]

  configured_params = PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'nav_params.yaml'])

  load_nodes = GroupAction(
        condition=IfCondition(NotSubstitution(use_composition)),
        actions=[
            PushRosNamespace(namespace),
            Node(
                condition=IfCondition(
                    AndSubstitution(
                        load_map_server,
                        EqualsSubstitution(LaunchConfiguration('map'), '')
                    )
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                condition=IfCondition(
                    AndSubstitution(
                        load_map_server,
                        NotEqualsSubstitution(LaunchConfiguration('map'), '')
                    )
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes_w_map_server}],
                condition=IfCondition(load_map_server)
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
                condition=UnlessCondition(load_map_server)
            ),                        
        ],
    )
  
  # == From nav2_bringup:
  # "LoadComposableNode for map server twice depending if we should use the
  # value of map from a CLI or launch default or user defined value in the
  # yaml configuration file. They are separated since the conditions
  # currently only work on the LoadComposableNodes commands and not on the
  # ComposableNode node function itself"
  load_composable_nodes = GroupAction(
      condition=IfCondition(use_composition),
      actions=[
          LoadComposableNodes(
              target_container=container_name,
              condition=IfCondition(
                  AndSubstitution(
                      load_map_server,
                      EqualsSubstitution(LaunchConfiguration('map'), '')
                  )
              ),
              composable_node_descriptions=[
                  ComposableNode(
                      package='nav2_map_server',
                      plugin='nav2_map_server::MapServer',
                      name='map_server',
                      parameters=[configured_params],
                      namespace=namespace,
                  ),
              ],
          ),
          LoadComposableNodes(
              target_container=container_name,
              condition=IfCondition(
                  AndSubstitution(
                      load_map_server,
                      NotEqualsSubstitution(LaunchConfiguration('map'), '')
                  )
              ),
              composable_node_descriptions=[
                  ComposableNode(
                      package='nav2_map_server',
                      plugin='nav2_map_server::MapServer',
                      name='map_server',
                      parameters=[
                          configured_params,
                          {'yaml_filename': map_yaml_file},
                      ],
                      namespace=namespace,
                  ),
              ],
          ),
          LoadComposableNodes(
              target_container=container_name,
              composable_node_descriptions=[
                  ComposableNode(
                      package='nav2_amcl',
                      plugin='nav2_amcl::AmclNode',
                      name='amcl',
                      parameters=[configured_params],
                      namespace=namespace,
                  ),
                  ComposableNode(
                      package='nav2_lifecycle_manager',
                      plugin='nav2_lifecycle_manager::LifecycleManager',
                      name='lifecycle_manager_localization',
                      parameters=[
                          {'autostart': autostart, 'node_names': lifecycle_nodes_w_map_server}
                      ],
                      namespace=namespace,
                      condition=IfCondition(load_map_server),
                  ),
                  ComposableNode(
                      package='nav2_lifecycle_manager',
                      plugin='nav2_lifecycle_manager::LifecycleManager',
                      name='lifecycle_manager_localization',
                      parameters=[
                          {'autostart': autostart, 'node_names': lifecycle_nodes}
                      ],
                      namespace=namespace,
                      condition=UnlessCondition(load_map_server),
                  ),
              ],
          ),
      ],
  )

  return LaunchDescription([
    *args,
    load_nodes,
    load_composable_nodes
  ])