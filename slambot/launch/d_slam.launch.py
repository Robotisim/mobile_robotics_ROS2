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
    return LaunchDescription([



        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slambot'),'launch', 'c_odom.launch.py')
        )),



        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),'launch', 'online_async_launch.py')
        )),



        # Node(
        #     package='cartographer_ros',
        #     output='screen',
        #     executable='cartographer_node',
        #     name='cr_node',
        #     arguments=['-configuration_directory', config_dir,'-configuration_basename','tb3.lua']
        # ),
        # Node(
        #     package='cartographer_ros',
        #     output='screen',
        #     executable='cartographer_occupancy_grid_node',
        #     name='cr_oc_node'
        # ),

    ])