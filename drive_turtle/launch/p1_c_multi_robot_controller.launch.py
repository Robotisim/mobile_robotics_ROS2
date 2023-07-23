from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Controller for turtle1
    turtle1_controller = Node(
        package='drive_turtle',
        executable='p1_multi_turtle_controller',
        name='turtle1_controller',
        parameters=[
            {'cmd_vel_topic': '/turtle1/cmd_vel'},
            {'linear_velocity': 2.0}
        ]
    )

    # Controller for turtle2
    turtle2_controller = Node(
        package='drive_turtle',
        executable='p1_multi_turtle_controller',
        name='turtle2_controller',
        parameters=[
            {'cmd_vel_topic': '/turtle2/cmd_vel'},
            {'linear_velocity': 1.5}
        ]
    )

    ld.add_action(turtle1_controller)
    ld.add_action(turtle2_controller)

    return ld
