from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    azrael_app_dir = get_package_share_directory('azrael_app')
    default_map_path = os.path.join(azrael_app_dir, 'map', 'cari_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map_yaml_file',
            default_value=default_map_path,
            description='Path to the map yaml file'
        ),

        # Map_server (lifecycle node)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_yaml_file')
            }]
        ),

        # Lifecycle manager 
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,  # Fa partire automaticamente configure + activate
                'node_names': ['map_server']
            }]
        )
    ])
