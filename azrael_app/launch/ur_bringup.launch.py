from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from moveit_configs_utils import MoveItConfigsBuilder
from launch.some_substitutions_type import SomeSubstitutionsType


AVAILABLE_GRIPPERS = ['robotiq-2f-85', 'robotiq-2f-140']

def launch_setup(context, *args, **kwargs):
    # Arguments passed to the robot description XACRO
    fake_ur = LaunchConfiguration('fake_ur')
    prefix = LaunchConfiguration('prefix')
    robot_ip = LaunchConfiguration('robot_ip')
    headless_mode = LaunchConfiguration('headless_mode')
    gripper = LaunchConfiguration('gripper')

    controller_spawner_timeout = LaunchConfiguration('controller_spawner_timeout')
    activate_joint_controller = LaunchConfiguration('activate_joint_controller')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')
    launch_dashboard_client = LaunchConfiguration('launch_dashboard_client')

    gripper_name = gripper.perform(context).replace('-', '_')

    srdf_path = PathJoinSubstitution([FindPackageShare('azrael_moveit_config'), 'config', f'azrael_{gripper_name}.srdf']).perform(context)
    joint_limits_path = PathJoinSubstitution([FindPackageShare('azrael_moveit_config'), 'config', 'joint_limits.yaml']).perform(context)
    moveit_controllers_path = PathJoinSubstitution([FindPackageShare('azrael_moveit_config'), 'config', 'moveit_controllers.yaml']).perform(context)

    robot_description_path = PathJoinSubstitution([FindPackageShare('azrael_description'), 'urdf', 'system.urdf.xacro']).perform(context)
    arm_description_path = PathJoinSubstitution([FindPackageShare('azrael_description'), 'urdf', 'azrael_arm.urdf.xacro'])
    robot_description_args : dict[SomeSubstitutionsType, SomeSubstitutionsType] = {
        'robot_ip' : robot_ip.perform(context),
        'fake_ur' : fake_ur.perform(context),
        'prefix' : f'{prefix.perform(context)}/',
        'gripper' : gripper.perform(context),
    }
    
    moveit_config = (
        MoveItConfigsBuilder('azrael', package_name='azrael_moveit_config')
        .robot_description(file_path=robot_description_path, mappings=robot_description_args)
        .robot_description_semantic(file_path=srdf_path)
        .planning_scene_monitor(publish_robot_description=False,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True)
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl', 'chomp'])
        .joint_limits(file_path=joint_limits_path)
        .trajectory_execution(file_path=moveit_controllers_path)
        .robot_description_kinematics()
        .to_moveit_configs()
    )
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    robot_description = moveit_config.robot_description
    arm_robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' ',
                arm_description_path,
                ' robot_ip:=', robot_ip,
                ' fake_ur:=', fake_ur,
                ' prefix:=', prefix, '/',
                ' gripper:=', gripper,
                ' generate_ros2_control_tag:=false',
            ]),
            value_type=str,
        )
    }

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare('azrael_app'), 'config', 'control_params.yaml']
    )

    # Define update rate for UR Robot
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare('azrael_app'),
            'config',
            'update_rate.yaml',
        ]
    )

    # UR Robot nodes
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output='screen',
        condition=IfCondition(fake_ur),
        # namespace='azrael'
    )

    ur_control_node = Node(
        package='ur_robot_driver',
        executable='ur_ros2_control_node',
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output='screen',
        condition=UnlessCondition(fake_ur),
    )

    dashboard_client_node = Node(
        package='ur_robot_driver',
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(fake_ur))
        ),
        executable='dashboard_client',
        name='dashboard_client',
        output='screen',
        emulate_tty=True,
        parameters=[{'robot_ip': robot_ip}],
    )

    urscript_interface = Node(
        package='ur_robot_driver',
        executable='urscript_interface',
        parameters=[{'robot_ip': robot_ip}],
        output='screen',
    )

    controller_stopper_node = Node(
        package='ur_robot_driver',
        executable='controller_stopper_node',
        name='controller_stopper',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(fake_ur),
        parameters=[
            {'headless_mode': headless_mode},
            {'joint_controller_active': activate_joint_controller},
            {
                'consistent_controllers': [
                    'io_and_status_controller',
                    'force_torque_sensor_broadcaster',
                    'joint_state_broadcaster',
                    'speed_scaling_state_broadcaster',
                    'ur_configuration_controller',
                    'admittance_controller',
                    'gpio_controller',
                ]
            },
        ],
    )

    # RSP
    arm_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='arm_robot_state_publisher',
        output='both',
        parameters=[arm_robot_description],
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ['--inactive'] if not active else []
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [ 'joint_state_broadcaster',
        # 'io_and_status_controller',
        # 'speed_scaling_state_broadcaster',
        'force_torque_sensor_broadcaster',
        'gpio_controller',
    ]
    controllers_inactive = [
        'forward_position_controller',
        'admittance_controller',
        # 'robotiq_action_controller',
        # 'robotiq_forward_command_controller',
    ]

    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    
    robotiq_controller_spawners = GroupAction(
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'gripper_activation_controller',
                    '-c',
                    '/controller_manager',
                    '--controller-manager-timeout',
                    controller_spawner_timeout,
                    '--param-file',
                    PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'gripper_control', f'gripper_controllers_{gripper_name}.yaml'])
                ],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'gripper_state_broadcaster',
                    '-c',
                    '/controller_manager',
                    '--controller-manager-timeout',
                    controller_spawner_timeout,
                    '--param-file',
                    PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'gripper_control', f'gripper_controllers_{gripper_name}.yaml'])
                ],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'gripper_controller',
                    '-c',
                    '/controller_manager',
                    '--controller-manager-timeout',
                    controller_spawner_timeout,
                    '--param-file',
                    PathJoinSubstitution([FindPackageShare('azrael_app'), 'config', 'gripper_control', f'gripper_controllers_{gripper_name}.yaml'])
                ]
            )
        ],
        condition=IfCondition(PythonExpression(['"', gripper, '" in ', repr(AVAILABLE_GRIPPERS)]))
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            initial_joint_controller,
            '-c',
            '/controller_manager',
            '--controller-manager-timeout',
            controller_spawner_timeout,
        ],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            initial_joint_controller,
            '-c',
            '/controller_manager',
            '--controller-manager-timeout',
            controller_spawner_timeout,
            '--inactive',
        ],
        condition=UnlessCondition(activate_joint_controller),
    )

    nodes_to_start = [
        move_group_node,
        control_node,
        ur_control_node,
        dashboard_client_node,
        controller_stopper_node,
        urscript_interface,
        arm_robot_state_publisher_node,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        *controller_spawners,
        robotiq_controller_spawners
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'fake_ur',
            default_value='true',
            description='Use fake hardware',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur10e',
            description='Type/series of used UR robot.',
            choices=['ur3', 'ur3e', 'ur5', 'ur5e', 'ur10', 'ur10e', 'ur16e', 'ur20', 'ur30'],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='azrael',
            description='prefix of the joint names.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.254.100',
            description='IP address by which the robot can be reached.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'headless_mode',
            default_value='true',
            description='Enable headless mode for robot control',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='100',
            description='Timeout used when spawning controllers.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'activate_joint_controller',
            default_value='true',
            description='Activate loaded joint controller.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='manipulator_controller',
            description='Initially loaded robot controller.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_dashboard_client',
            default_value='true',
            description='Launch Dashboard Client?'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper',
            default_value='None',
            description='Gripper mounted',
            choices=['None', *AVAILABLE_GRIPPERS],
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
