from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim1',
        namespace='robot1'
    )

    turtlesim2 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim2',
        namespace='robot2'
    )

    ld.add_action(turtlesim1)
    ld.add_action(turtlesim2)

    return ld
