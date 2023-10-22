import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.actions import TimerAction

def generate_launch_description():

    ld = LaunchDescription()

    rviz_config_path = os.path.join(
        get_package_share_directory('py_slambot'),  # Replace with your package name
        'rviz',
        'odom_view.rviz'
    )

    rviz2= Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='log'
    )

    odom_from_enc= Node(
            package='py_slambot',
            executable='odom_from_enc',
            name='odom_from_enc'
    )



    ld.add_action(rviz2)
    ld.add_action(odom_from_enc)

    return ld