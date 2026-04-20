""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: azrael/base_link -> camera_link """
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
                "azrael/base_link",
                "--child-frame-id",
                "camera_link",
                "--x",
                "-0.0198132",
                "--y",
                "0.252878",
                "--z",
                "0.0375784",
                "--qx",
                "-0.00608862",
                "--qy",
                "-0.000572681",
                "--qz",
                "0.706174",
                "--qw",
                "0.708012",
                # "--roll",
                # "3.13378",
                # "--pitch",
                # "-3.13218",
                # "--yaw",
                # "-1.57343",
            ],
        ),
    ]
    return LaunchDescription(nodes)
