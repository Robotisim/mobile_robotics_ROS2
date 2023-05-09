from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Controller for turtle1
    robot_a_controller = Node(
        package='drive_tb3',
        executable='robot_square',
        name='robot_a_controller',
        parameters=[
            {'cmd_vel_topic': '/robot_a/cmd_vel'}
        ]
    )

    robot_b_controller = Node(
        package='drive_tb3',
        executable='robot_square',
        name='robot_b_controller',
        parameters=[
            {'cmd_vel_topic': '/robot_b/cmd_vel'}
        ]
    )
    robot_c_controller = Node(
        package='drive_tb3',
        executable='robot_square',
        name='robot_c   _controller',
        parameters=[
            {'cmd_vel_topic': '/robot_c/cmd_vel'}
        ]
    )

    ld.add_action(robot_a_controller)
    ld.add_action(robot_b_controller)
    ld.add_action(robot_c_controller)

    return ld
