#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
############# Paths Setting
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_tb3_drive = get_package_share_directory('drive_tb3')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    maze_path = os.path.join(pkg_tb3_drive,'models','maze','model.sdf')


    config_dir = os.path.join(get_package_share_directory('turtlebot3_navigation2'),'config')
    rviz_config= os.path.join(config_dir,'tb3_navigation2.rviz')

    map_file = os.path.join(pkg_tb3_drive,'map','maze_map.yaml')
    params_file = os.path.join(pkg_tb3_drive,'map','tb3_nav_params.yaml')
############# Gazebo Definations

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
############# Spawning Robot and Maze
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    robot_spawner = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_tb3_drive, 'launch', 'p2_a_spawn_tb3.launch.py')
    ),
    launch_arguments={
        'x_pos': '-6.1',
        'y_pos': '-7.1',
        'yaw_rot': '1.57',
        'robot_name' :'turtlebot_3',
    }.items()
)
    maze_spawner = Node(
        package='drive_tb3',
        executable='p6_b_sdf_spawner',
        name='maze_model',
        arguments=[maze_path,"Maze","0.0","0.0"]

    )
############# Mapping Functionality
    maze_mapping = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_toolbox'),'launch', 'online_async_launch.py')
        ),
    )


############ Rviz Configuration

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            # parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
    )
# Add 2 more Nodes for spawning robot with navigation package

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_spawner)
    ld.add_action(maze_spawner)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(maze_mapping)
    ld.add_action(rviz)

    return ld
