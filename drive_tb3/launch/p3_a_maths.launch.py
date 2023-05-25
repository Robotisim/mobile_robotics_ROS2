#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tb3_drive = get_package_share_directory('turtlebot3_gazebo')

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'empty_world.launch.py')
        ),

    )
    reconfigure = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='parameter_reconfig'

    )

    drive_tb3_b = Node(
        package='drive_tb3',
        executable='p3_d_xy_goal',
        name='custom_node'
        # parameters=[
        #     {'cmd_vel_topic': '/robot_b/cmd_vel'},
        # ]

    )

    ld = LaunchDescription()
    ld.add_action(spawn_tb3)
    ld.add_action(reconfigure)
    ld.add_action(drive_tb3_b)

    return ld
