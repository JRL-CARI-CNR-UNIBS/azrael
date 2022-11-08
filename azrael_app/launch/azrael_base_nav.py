#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import LogInfo
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace



def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="azrael_description",
            description="mobile manip description",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="system.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level'))

    log_level                = LaunchConfiguration('log_level')
    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            # " ","safety_limits:=",safety_limits,
            # " ","safety_pos_margin:=",safety_pos_margin,
            # " ","safety_k_position:=",safety_k_position,
            " ","name:=","azrael",
            " ","ur_type:=","ur10",
            " ","prefix:=","azrael/",
            # " ","prefix_rc:=","azrael",
            # " ","simulation_controllers:=",initial_joint_controllers_1,
            # " ","use_fake_hardware:=",use_fake_hardware,
            # " ","sim_gazebo:=",sim_gazebo,
        ]
    )

    robot_description_1  = {"robot_description": robot_description_content}
    frame_prefix_param_1 = {"frame_prefix": ""}


    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace='azrael',
        output="log",
        parameters=[robot_description_1,frame_prefix_param_1],
    )


    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        # namespace='azrael',
        executable="joint_state_publisher",
        output="screen",
        remappings=[
            ("/robot_description", "/azrael/robot_description")
        ]
    )


    ##MAP

    map_yaml_file = os.path.join(
            get_package_share_directory('azrael_description'),
            'map',
            'map.yaml')

    param_map = {'yaml_filename': map_yaml_file}

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='azrael',
            output='screen',
            parameters=[param_map],
            )

    ##AMCL

    amcl_params = os.path.join(
            get_package_share_directory('azrael_app'),
            'config',
            "amcl.yaml")

    configured_params_amcl = RewrittenYaml(
            source_file=amcl_params,
            root_key="azrael",
            param_rewrites={},
            convert_types=True)

    amcl_node =  Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace='azrael',
                output='screen',
                parameters=[configured_params_amcl]
                )

    ##NAVIGATION

    nav_sw1_params = os.path.join(
            get_package_share_directory('azrael_app'),
            'config',
            "nav_params.yaml")

    configured_params = RewrittenYaml(
        source_file=nav_sw1_params,
        root_key="azrael",
        param_rewrites={'autostart': "True"},
        convert_types=True)

    # lifecycle_nodes = ['controller_server',
    #                    'smoother_server',
    #                    'planner_server',
    #                    'behavior_server',
    #                    'bt_navigator',
    #                    'map_server',
    #                    'amcl'
    #                    'waypoint_follower']
    #                 #    'velocity_smoother']

    lifecycle_nodes = ['map_server',
                       'amcl',
                       ]

    load_nodes = GroupAction(
        actions=[PushRosNamespace('azrael'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings= [('cmd_vel', '/azrael/cmd_vel')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                ),
            # Node(
            #     package='nav2_velocity_smoother',
            #     executable='velocity_smoother',
            #     name='velocity_smoother',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings +
            #             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        ]
    )

    lf_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace='azrael',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}])

    nodes_to_start = [
        robot_state_publisher_node_1,
        # joint_state_publisher_node,
        map_server,
        amcl_node,
        lf_manager,
        # TimerAction(
        #         period=2.0,
        #         actions=[load_nodes],
        #         ),

    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

