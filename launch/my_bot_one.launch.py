import os
from ament_index_python.packages import get_package_share_directory 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    package_name = 'my_bot_one'
    
    # Params files
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'gz_bridge.yaml'
    )
    
    joy_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'xbox.config.yaml'
    )

    # Include robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Robot description (from Xacro)
    #robot_description_content = Command([
     #   'xacro ',
      #  PathJoinSubstitution([
       #     FindPackageShare(package_name),
        #    'description',
         #   'robot.urdf.xacro'
      #  ])
    #])


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
    
    # Controller parameters
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ controller_params_file],
        output='screen'
    )
    
    #delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # DiffDrive controller spawner
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    # Joint State Broadcaster
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    # ROS-Gazebo Bridge
    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        ros_gz_bridge
    ])

    

    