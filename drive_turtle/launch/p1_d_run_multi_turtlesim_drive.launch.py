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
    pkg_path = get_package_share_directory('drive_turtle')

    bring_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'p1_b_multi_robot_single_sim.launch.py')
        ),
    )

    bring_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'p1_c_multi_robot_controller.launch.py')
    ),
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(bring_sim)
    ld.add_action(bring_robots)

    return ld
