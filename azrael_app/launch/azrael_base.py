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


    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")


    azrael_driver_udp = Node(
        package="azrael_driver_udp",
        namespace='azrael',
        executable="azrael_driver_udp_node",
        output="log")

    rplidar = Node(
        package='rplidar_ros2',
        executable='rplidar_scan_publisher',
        namespace='azrael',
        name='rplidar_scan_publisher',
        parameters=[{'serial_port': "/dev/ttyUSB0", 
                        'serial_baudrate': 256000, 
                        'frame_id': "azrael/laser",
                        'inverted': False, 
                        'angle_compensate': True, 
                        'channel_type': "",
                        'tcp_ip': "1.1.1.1",
                        'udp_ip': "1.1.1.1",
                        'tcp_port': 11,
                        'udp_port': 11,
                        'scan_frequency': 10.0,
                        'scan_mode': "Sensitivity"}],

        output='log')

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


    map_yaml_file = os.path.join(
            get_package_share_directory('azrael_description'),
            'map',
            'map.yaml')

    param_map = {'yaml_filename': map_yaml_file}

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            # namespace='azrael',
            output='screen',
            parameters=[param_map])


    amcl_params = os.path.join(
            get_package_share_directory('azrael_app'),
            'config',
            "amcl.yaml")

    configured_params = RewrittenYaml(
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
                parameters=[configured_params],
                remappings=[
                    ("map", "/map")
                ]
                )

    # lifecycle_nodes = ['map_server', '/azrael/amcl']
    lifecycle_nodes_map  = ['map_server']
    lifecycle_nodes_amcl = ['amcl']

    life_mg_map = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'bond_timeout': 10.0},
                            {'node_names': lifecycle_nodes_map}])

    life_mg_amcl = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace='azrael',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'bond_timeout': 10.0},
                            {'node_names': lifecycle_nodes_amcl}])

    nodes_to_start = [
        rplidar,
        azrael_driver_udp,
        robot_state_publisher_node_1,
        # joint_state_publisher_node,
        map_server,
        amcl_node,
        TimerAction(
            period=2.0,
            actions=[life_mg_map],
        ),
        TimerAction(
            period=5.0,
            actions=[life_mg_amcl],
        )
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

