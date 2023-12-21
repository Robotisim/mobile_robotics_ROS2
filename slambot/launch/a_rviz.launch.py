import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'package_name'
    pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s && pwd"' % pkg_name).read().strip()

    return LaunchDescription([

        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(pkg_dir, 'config', 'config_file.rviz')]]
        )


    ])