#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the rplidar_laser_scan.launch.py
    rplidar_laser_scan = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')

    # Arguments for the included launch file
    frame_id_arg = LaunchConfiguration('frame_id', default='lidar_frame')

    return LaunchDescription([
        # Declare the arguments
        DeclareLaunchArgument('frame_id', default_value='lidar_frame', description='For Proper Transforms'),

        # Include the rplidar_laser_scan.launch.py with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_laser_scan),
            launch_arguments={'frame_id': frame_id_arg}.items()
        ),

        # Start the occupancy_grid node from py_slambot package
        Node(
            package='py_slambot',
            executable='occupancy_grid',
            name='occupancy_grid_node',
            output='screen'
        )
    ])
