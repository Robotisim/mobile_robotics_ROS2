from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_slambot',
            executable='robot_driving',
            name='robot_drive_node',
            output='screen'
        ),
        Node(
            package='py_slambot',
            executable='encoders_publish',
            name='encoder_publisher_node',
            output='screen'
        ),
    ])
