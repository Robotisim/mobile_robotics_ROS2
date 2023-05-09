from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
def generate_launch_description():
    ld = LaunchDescription()

    bring_sim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    kill_first = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/kill', 'turtlesim/srv/Kill', "\"{name: 'turtle1'}\"",],
        name='spawn_turtle1',
        shell=True
    )

    spawn_turtle_1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 3.0, y: 1.0, theta: 1.57, name: 'turtle1'}\"",],
        name='spawn_turtle1',
        shell=True
    )

    spawn_turtle1_delayed = TimerAction(
            period=2.0,
            actions=[spawn_turtle_1],
        )

    spawn_turtle_2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 2.0, y: 1.0, theta: 1.57, name: 'turtle2'}\"",],
        name='spawn_turtle1',
        shell=True
    )

    ld.add_action(bring_sim)
    ld.add_action(kill_first)
    ld.add_action(spawn_turtle1_delayed)
    ld.add_action(spawn_turtle_2)

    return ld
