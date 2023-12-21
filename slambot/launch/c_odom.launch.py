import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # pkg_name = 'package_name'
    # pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
    #     colcon_cd %s && pwd"' % pkg_name).read().strip()

    return LaunchDescription([

        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(pkg_dir, 'config', 'config_file.rviz')]]
        # )

        # Node(
        #     package='slambot',
        #     executable='p7_a_drive_control.py',
        #     name='encoder_parser',
        # ),

        Node(
            package='slambot',
            executable='p7_c_odom.py',
            name='odom_generator',
        ),


                # static transform for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.064', '0', '0.120', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        # static transform from link to footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '-0.0325', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),

        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Boost',
            }],
        )

    ])