#!/usr/bin/env python3
# Combining world file with 3 calls to spawn turtle3 file Tool

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('drive_tb3'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ld = LaunchDescription()
    ########################## Launching Gazebo iwth world ##########################
    world = os.path.join(
        get_package_share_directory('drive_tb3'),
        'world',
        'sensor_test_track.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ########################## Launching Robots  ##########################
    tb3_1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'p2_a_spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '-7.9',
            'y_pose': '-8.76',
            'cc_rot': '1.57',
            'robot_name'  : 'waffle_a',
            'robot_ns'  : 'robot_a'
        }.items()
    )




    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(tb3_1_spawn)
    return ld
