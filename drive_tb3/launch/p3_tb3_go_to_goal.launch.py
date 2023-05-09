from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    ld = LaunchDescription()
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'empty_world.launch.py')
        )

    )

    gtg_node = Node(
        package='drive_tb3',
        executable='p3_go_to_goal',
        name='robot_a_controller',
        parameters=[
            {'goal_x': '4.0',
            'goal_y': '4.0',
            'kp': '3.0'

             }
        ]
    )

    ld.add_action(robot_spawn)
    ld.add_action(gtg_node)

    return ld
