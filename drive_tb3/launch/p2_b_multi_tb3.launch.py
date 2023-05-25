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
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_drive = get_package_share_directory('drive_tb3')

    #### Gazebo Launching and importing
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
    ### Robot Spawning
    robot_spawner1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'2.0' ,
            'y_pos' :'-2.0',
            'yaw_rot' :'1.57',
            'robot_name' :'tb_1',
            'robot_ns'  : 'robot_a'

        }.items()
    )

    robot_spawner2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'2.0' ,
            'y_pos' :'2.0',
            'yaw_rot' :'3.14',
            'robot_name' :'tb_2',
            'robot_ns'  : 'robot_b'

        }.items()
    )

    robot_spawner3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'-2.0' ,
            'y_pos' :'2.0',
            'yaw_rot' :'-1.57',
            'robot_name' :'tb_3',
            'robot_ns'  : 'robot_c'

        }.items()
    )

    robot_spawner4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos' :'-2.0' ,
            'y_pos' :'-2.0',
            'yaw_rot' :'0.0',
            'robot_name' :'tb_4',
            'robot_ns'  : 'robot_d'

        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_spawner1)
    ld.add_action(robot_spawner2)
    ld.add_action(robot_spawner3)
    ld.add_action(robot_spawner4)

    return ld
