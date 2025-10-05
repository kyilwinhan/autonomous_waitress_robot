from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch
import launch_ros.actions


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    config_filepath = LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='/diff_cont/cmd_vel'),
        # default points to a file name inside config/
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox.config.yaml'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='0'),
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='true'),

        launch.actions.DeclareLaunchArgument(
            'config_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_bot_one'),
                'config',
                joy_config,   # use substitution, not string literal
            ])
        ),

        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': LaunchConfiguration('joy_dev'),
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }, config_filepath]
        ),

        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath, {'publish_stamped_twist': LaunchConfiguration('publish_stamped_twist')}],
            remappings=[('/cmd_vel', LaunchConfiguration('joy_vel'))]
        ),
    ])
