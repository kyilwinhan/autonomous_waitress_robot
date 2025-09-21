import os
from ament_index_python.packages import get_package_share_directory 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource 

from launch_ros.actions import Node 

def generate_launch_description():

    #include the robot_state_publisher launch file,provided by our own package. Force sim time to be enabled 
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!! .

    package_name = 'my_bot_one'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory(package_name), 'launch', 'rsp.launch.py'

    )]), launch_arguments={'use_sim_time': 'true'}.items()
)
    # Include the Gazebo launch file, 

    # Run the spawner node from gazebo

    # Lauch them all

    

    