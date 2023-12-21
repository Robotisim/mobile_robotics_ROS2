# ROS2 Mobile Robotics
This repository will be carrying code for 2D Slam ros2 based Robot in CPP .

## Setup RPI for ROS2 Development
- Install RPI Imager → Ubuntu 22.04 Server
    - Setup Wifi network file change ssid and password → no need of HDMI display
- Vscode SSH → find ip using network tools android app
- SSH username : ubuntu
- SSH Password : ubuntu

### Running
- Clone this package on both the RPI and PC
- To View Odometery on RVIZ
    ```
    #on RPI
    ros2 launch py_slambot rpi_a_robot_drive_enc_pub.launch.py

    #on PC
    ros2 launch py_slambot pc_a_odom-view.launch.py
    ```
- View Lidar
    ```

    ```

### Remaning
- Mapping from Lidar

## Requirements
### Software
- Ubuntu 22.04 , Ubuntu 22 Server
- ROS2 Humble
- Vscode

### Hardware
- Raspberry Pi 4B
- PowerBank 3A
- 2x DC Motors
- 2x Wheels
- 1x Caster Wheel
- 1x L298N Motor Driver
- 1x 12V Battery
