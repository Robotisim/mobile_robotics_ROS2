from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'py_slambot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoders_publish = py_slambot.p7_a_encoders_ros:main',
            'robot_driving = py_slambot.p7_a_cmd_vel_ros:main',
            'odom_from_enc = py_slambot.p7_a_odom:main',
            'occupancy_grid = py_slambot.p7_b_occupancy_grid:main',
        ],
    },
)
