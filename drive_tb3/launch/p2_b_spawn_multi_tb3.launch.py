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
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_dqn_stage1.world'
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
            'x_pose': '0.0',
            'y_pose': '-.5',
            'robot_name'  : 'waffle_a',
            'robot_ns'  : 'robot_a'
        }.items()
    )

    tb3_2_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'p2_a_spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'robot_name'  : 'waffle_b',
            'robot_ns'  : 'robot_b'
        }.items()
    )
    tb3_3_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'p2_a_spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.50',
            'robot_name'  : 'waffle_c',
            'robot_ns'  : 'robot_c'
        }.items()
    )



    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(tb3_1_spawn)
    ld.add_action(tb3_2_spawn)
    ld.add_action(tb3_3_spawn)
    return ld
