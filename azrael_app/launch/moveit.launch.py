from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
  launch_args = [
    DeclareLaunchArgument(name="robot_ip", description="ur net IP"),
    DeclareLaunchArgument(name="use_fake_hardware", default_value="false", description="use fake hardware"),
    DeclareLaunchArgument(name="ft_sensor_ros2_control", default_value="true", description="load ros2_control config of ft_sensor"),
    DeclareLaunchArgument(name="rviz", default_value="true", description="Load Rviz"),
  ]
  return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):

  robot_description_path = PathJoinSubstitution([FindPackageShare("azrael_description"), "urdf", "system.urdf.xacro"]).perform(context)
  robot_description_args = {
    "robot_ip" : LaunchConfiguration("robot_ip"),
    "use_fake_hardware" : LaunchConfiguration("use_fake_hardware"),
    "ft_sensor_ros2_control" : LaunchConfiguration("ft_sensor_ros2_control"),
  }

  srdf_path = PathJoinSubstitution([FindPackageShare("azrael_moveit_config"), "config", "azrael.srdf"]).perform(context)
  joint_limits_path = PathJoinSubstitution([FindPackageShare("azrael_moveit_config"), "config", "joint_limits.yaml"]).perform(context)
  moveit_controllers_path = PathJoinSubstitution([FindPackageShare("azrael_moveit_config"), "config", "moveit_controllers.yaml"]).perform(context)

  rviz_config_path = PathJoinSubstitution([FindPackageShare("azrael_moveit_config"), "config", "moveit.rviz"])

  moveit_config = (
    MoveItConfigsBuilder("azrael", package_name="azrael_moveit_config")
    .robot_description(file_path=robot_description_path, mappings=robot_description_args)
    .robot_description_semantic(file_path=srdf_path)
    .planning_scene_monitor(publish_robot_description=True)
    .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
    .joint_limits(file_path=joint_limits_path)
    .trajectory_execution(file_path=moveit_controllers_path)
    .to_moveit_configs()
  )

  # print(moveit_config.robot_description["robot_description"].value[0].perform(context))

  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    condition=IfCondition(LaunchConfiguration("rviz")),
    parameters=[
      moveit_config.to_dict()
    ],
    arguments=["-d", rviz_config_path],
  )

  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
    parameters=[moveit_config.robot_description]
  )

  return [
    move_group_node,
    rviz_node,
    robot_state_publisher_node,
  ]