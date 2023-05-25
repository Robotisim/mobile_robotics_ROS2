#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
############# Paths Setting

    pkg_tb3_drive = get_package_share_directory('drive_tb3')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(
        pkg_tb3_drive,
        'worlds',
        'line_following.world'
    )
############# Gazebo Definations

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
############# Spawning Robot
    robot_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
        ),
        launch_arguments={
            'x_pos': '2.16',
            'y_pos': '-1.04',
            'yaw_rot': '3.11',
            'robot_name' :'turtlebot_3',
            # 'robot_ns'  : 'line_follower'

        }.items()
    )
############# Lidar Processing Node

    follow_line_tb3 = Node(
        package='drive_tb3',
        executable='p5_c_camera_data_sub',
        name='Camera_processing'
        # parameters=[
        #     {'cmd_vel_topic': '/robot_b/cmd_vel'},
        # ]

    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_spawner)
    # ld.add_action(follow_line_tb3)

    return ld
