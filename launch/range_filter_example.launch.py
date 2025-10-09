from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="laser_filters",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("my_bot_one"),
                    "config", "lidar_filter.yaml",
                ])],
            remappings=[
                ("scan", "/scan_raw"),      # input from lidar driver
                ("scan_filtered", "/scan_filtered"), # output to be used by SLAM or Nav2
            ]
        )
    ])
