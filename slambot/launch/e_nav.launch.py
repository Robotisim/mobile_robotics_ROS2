from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('slambot'),'config')
    map_file = os.path.join(get_package_share_directory('slambot'),'map', 'map.yaml')
    params_file = os.path.join(config_dir,'nav_params.yaml')

    return LaunchDescription([



        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slambot'),'launch', 'd_slam.launch.py')
        )),



        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file}.items(),

    )




    ])