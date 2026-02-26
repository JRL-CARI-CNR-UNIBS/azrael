""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: azrael/tool0 -> camera_link """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "azrael/tool0",
                "--child-frame-id",
                "azrael_camera_link",
                "--x",
                "0.00652052",
                "--y",
                "-0.0466509",
                "--z",
                "0.0420558",
                "--qx",
                "-0.514188",
                "--qy",
                "0.524941",
                "--qz",
                "-0.50627",
                "--qw",
                "-0.451375",
                # "--roll",
                # "1.65087",
                # "--pitch",
                # "0.0467622",
                # "--yaw",
                # "1.63465",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_azrael_base_to_marker",
            arguments=[
                # Traslazione (m)
                "--x", "0.082", # 0.042, 1.036, 0.457
                "--y", "0.086",
                "--z", "0.026",
                # Rotazione (quaternione xyzw)
                "--qx", "0.678",
                "--qy", "0.735",
                "--qz", "0.028",
                "--qw", "-0.016",
                # Frame parent e child
                "--frame-id", "aruco_marker_0",
                "--child-frame-id", "box_pick",
            ],
        )
    ]
    return LaunchDescription(nodes)
