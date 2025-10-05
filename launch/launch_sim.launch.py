import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot_one'

    # Arguments
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory(package_name), 'worlds', '2block_world.sdf'
        ),
        description='World to load'
    )

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

    # --- Simulation group (Gazebo + world + bridge) ---
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
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', package_name, '-z', '0.1'],
            output='screen'
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ]
        )
    ])

    # --- Robot description group (RSP + controllers) ---
    robot_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
        ),
        Node(
            package='controller_manager',
            executable="spawner",
            arguments=["diff_cont"],
            
        ),
        Node(
            package='controller_manager',
            executable="spawner",
            arguments=["joint_broad"]
        ),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output='screen',
            parameters=[twist_mux_params, {'use_sim_time': True}, {'use_stamped': True}],
            remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params, {'use_sim_time': True}
                ]
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_params, {'publish_stamped_twist':True}, {'use_sim_time': True}],
            remappings=[('/cmd_vel', '/cmd_vel_joy')]
        ),
        
        Node(
             package='twist_stamper',
             executable='twist_stamper',
             parameters=[{'use_sim_time': True}],
             remappings=[('/cmd_vel_in','/cmd_vel'),
                         ('/cmd_vel_out','/diff_cont/cmd_vel')]
          )
    ]) 

    # --- Visualization group (RViz2) ---
    visualization_group = GroupAction([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory(package_name),
                'config',
                'fully_loaded_my_bot_one.rviz')],
            parameters=[{'use_sim_time': True}]
        )
    ])

    # --- Final LaunchDescription ---
    return LaunchDescription([
        world_arg,
        simulation_group,
        robot_group,
        visualization_group
    ])
