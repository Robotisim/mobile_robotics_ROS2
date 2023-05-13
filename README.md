# ROS2 Mobile Robotics
This repository will be carrying code for basics of mobile robotics with ros2 implementation in CPP .

## Repository Structure for ROS2 Course
- **drive_turtle**     : Multi-turtlesim driving with ROS2
    - Learnings : Topics , Nodes , Launch Files , Services
    - P1 :
- **drive_turtlebot3** : Turtlebot3 driving with ROS2
    - Namespaces , Launch arguments , topics remapping
    - P2 : run p2_b and p2_c_sqaure.launch to move multi turtles in square
    - ros2 topic pub --once /robot_a/cmd_vel geometry_msgs/msg/Twist "{'linear': {'x': 0.5, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.1}}"
    - ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_a/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_a/cmd_vel




## Requirements
### Software
- Ubuntu 22.04
- ROS2 Humble
- Vscode

### Hardware
- Raspberry Pi 3B+
- Arduino Uno
- 2x DC Motors
- 2x Wheels
- 1x Caster Wheel
- 1x L298N Motor Driver
- 1x 12V Battery
