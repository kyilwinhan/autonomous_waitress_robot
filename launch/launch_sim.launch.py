import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    package_name = 'autonomous_waitress_robot'

    # --- Launch arguments ---
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory(package_name), 'worlds', 'world_for_simulation_robot.sdf'
        ),
        description='World to load in Gazebo'
    )

    # --- Config files ---
    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'gazebo_params.yaml'
    )
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )
    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml'
    )
    joy_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'xbox.config.yaml'
    )

    # --- Simulation: Gazebo + World + Bridge ---
    create_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', package_name, '-z', '0.1'],
        output='screen'
    )

    simulation_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': ['-r -v4 ', world],
                'on_exit_shutdown': 'true',
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
            }.items(),
        ),
        create_robot_node,
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen'
        )
    ])

    # --- Robot Description + Controllers ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true', 'sim_mode': 'true'}.items()
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    diff_cont_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    # 🧠 Delay controllers until robot is spawned in Gazebo
    delay_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=create_robot_node,
            on_start=[joint_broad_spawner, diff_cont_spawner]
        )
    )

    # --- Teleop + Twist Mux ---
    teleop_and_joy = [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params, {'use_sim_time': True}]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_params, {'publish_stamped_twist': True}, {'use_sim_time': True}],
            remappings=[('/cmd_vel', '/cmd_vel_joy')]
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_params, {'use_sim_time': True}, {'use_stamped': True}],
            remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
        ),
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': True}],
            remappings=[('/cmd_vel_in', '/cmd_vel'),
                        ('/cmd_vel_out', '/diff_cont/cmd_vel')]
        )
    ]

    # --- Visualization (RViz2) ---
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(package_name),
            'config',
            'fully_loaded_autonomous_waitress_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- Final LaunchDescription ---
    return LaunchDescription([
        world_arg,
        simulation_group,
        rsp,
        delay_controllers,
        *teleop_and_joy,
        rviz
    ])
