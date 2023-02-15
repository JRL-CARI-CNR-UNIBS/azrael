#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import LogInfo
from launch_ros.actions import Node



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



    azrael_driver_udp = Node(
        package="azrael_driver_udp",
        namespace='azrael',
        executable="azrael_driver_udp_node",
        output="log")

    # rplidar =   Node(
    #         name='sllidar_ros2',
    #         package='sllidar_ros2',
    #         namespace='azrael',
    #         executable='sllidar_node',
    #         output='screen',
    #         parameters=[{
    #             'serial_port': '/dev/ttyUSB0',
    #             'serial_baudrate': 256000,  # A3
    #             'frame_id': 'azrael/laser',
    #             'inverted': False,
    #             'angle_compensate': True,
    #             'scan_mode': 'Sensitivity',
    #         }],
    #     )

    # sick_scan_pkg_prefix = get_package_share_directory('sick_scan')
    # launchfile = os.path.basename(__file__)[:-3] # convert "<lidar_name>.launch.py" to "<lidar_name>.launch"
    # launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/' + launchfile) # 'launch/sick_lms_1xx.launch')
    # node_arguments=[launch_file_path]


    sick = Node(
            package='sick_scan',
            executable='sick_generic_caller',
            namespace='azrael',
            output='screen',
            remappings= [('/azrael/sick_lms_1xx/scan', '/azrael/scan')],
            parameters=
            [{"intensity"                           : False},
            {"intensity_resolution_16bit"           : False},
            {"min_ang"                              : -2.35619},
            {"max_ang"                              : 2.35619},
            {"frame_id"                             :"azrael/laser"},
            {"use_binary_protocol"                  : True},
            {"scanner_type"                         :"sick_lms_1xx"},
            {"hostname"                             :"192.170.1.1"},
            {"cloud_topic"                          :"cloud"},
            {"port"                                 :"2112"},
            {"timelimit"                            : 5},
            {"min_intensity"                        : 0.0},
            {"use_generation_timestamp"             : True},
            {"range_min"                            : 0.05},
            {"range_max"                            : 25.0},
            {"scan_freq"                            : 50.0},
            {"ang_res"                              : 0.5},
            {"range_filter_handling"                : 0},
            {"add_transform_xyz_rpy"                : "0,0,0,0,0,0"},
            {"add_transform_check_dynamic_updates"  : False},
            {"start_services"                       : True},
            {"message_monitoring_enabled"           : True},
            {"read_timeout_millisec_default"        : 5000},
            {"read_timeout_millisec_startup"        : 120000},
            {"read_timeout_millisec_kill_node"      : 15000000},
            {"client_authorization_pw"              :"F4724744"},
            {"imu_enable"                           : False},
            {"ros_qos"                              : 4}])


    robot_description_1  = {"robot_description": robot_description_content}
    frame_prefix_param_1 = {"frame_prefix": ""}


    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace='azrael',
        output="log",
        parameters=[robot_description_1,frame_prefix_param_1])

    nodes_to_start = [
        # rplidar,
        sick,
        azrael_driver_udp,
        robot_state_publisher_node_1
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

