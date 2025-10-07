import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, GroupAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    package_name = 'my_bot_one'

    # --- Launch arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    sim_mode = LaunchConfiguration('sim_mode', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true'
    )
    declare_sim_mode = DeclareLaunchArgument(
        'sim_mode', default_value='true', description='Simulation mode true/false'
    )
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control', default_value='true', description='Use ros2_control if true'
    )

    # --- Config paths ---
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    joy_params = os.path.join(get_package_share_directory(package_name), 'config', 'xbox.config.yaml')
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    # --- Robot description from xacro ---
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(package_name),
            'description',
            'robot.urdf.xacro'
        ]),
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', sim_mode
    ])

    # --- ROS-Gazebo bridge ---
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # --- Controller manager node ---
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params_file],
        output='screen'
    )

    # --- Spawners for controllers ---
    diff_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_cont'],
                output='screen'
            )]
        )
    )

    joint_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broad'],
                output='screen'
            )]
        )
    )

    # --- Robot State Publisher and other nodes ---
    robot_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_ros2_control': use_ros2_control
            }.items()
        ),

        # Twist Mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_params, {'use_sim_time': False}, {'use_stamped': True}],
            remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
        ),

        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_params, {'use_sim_time': False}]
        ),

        # Teleop Node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_params, {'publish_stamped_twist': True}, {'use_sim_time': False}],
            remappings=[('/cmd_vel', '/cmd_vel_joy')]
        ),

        # Twist Stamper to convert Twist → TwistStamped
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[('/cmd_vel_in', '/cmd_vel'),
                        ('/cmd_vel_out', '/diff_cont/cmd_vel')]
        ),
    ])

    # --- Final LaunchDescription ---
    return LaunchDescription([
        declare_use_sim_time,
        declare_sim_mode,
        declare_use_ros2_control,
        ros_gz_bridge,
        controller_manager_node,
        diff_spawner,
        joint_spawner,
        robot_group
    ])
