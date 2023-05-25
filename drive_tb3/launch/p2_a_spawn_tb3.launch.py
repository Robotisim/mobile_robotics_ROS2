#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # File Paths are stores
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Define arguments - parameters
    x_pos = LaunchConfiguration('x_pos',default='0.0')
    y_pos = LaunchConfiguration('y_pos',default='0.0')
    yaw_rot = LaunchConfiguration('yaw_rot',default='0.0')
    robot_name = LaunchConfiguration('robot_name',default='waffle_pi')
    robot_ns = LaunchConfiguration('robot_ns',default='')


    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', urdf_path,
            '-x', x_pos,
            '-y', y_pos,
            '-z', '0.01',
            '-Y',yaw_rot,
            '-robot_namespace',robot_ns
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
