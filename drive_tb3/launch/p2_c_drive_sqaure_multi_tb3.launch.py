#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tb3_drive = get_package_share_directory('drive_tb3')

    spawn_multi_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_b_multi_tb3.launch.py')
        ),

    )
    drive_tb3_a = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_a',
        parameters=[
            {'cmd_vel_topic': '/robot_a/cmd_vel'},
        ]

    )

    drive_tb3_b = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_b',
        parameters=[
            {'cmd_vel_topic': '/robot_b/cmd_vel'},
        ]

    )
    drive_tb3_c = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_c',
        parameters=[
            {'cmd_vel_topic': '/robot_c/cmd_vel'},
        ]

    )
    drive_tb3_d = Node(
        package='drive_tb3',
        executable='p2_multi_tb3_sqaure_drive',
        name='drive_tb3_d',
        parameters=[
            {'cmd_vel_topic': '/robot_d/cmd_vel'},
        ]

    )
    ld = LaunchDescription()
    ld.add_action(spawn_multi_tb3)
    ld.add_action(drive_tb3_a)
    ld.add_action(drive_tb3_b)
    ld.add_action(drive_tb3_c)
    ld.add_action(drive_tb3_d)

    return ld
