# ROS2 Mobile Robotics
This repository will be carrying code for 2D Slam ros2 based Robot in CPP .

## Setup RPI for ROS2 Development
- Install RPI Imager → Ubuntu 22.04 Server
    - Setup Wifi network file change ssid and password → no need of HDMI display
- Vscode SSH → find ip using network tools android app
- SSH username : ubuntu
- SSH Password : ubuntu
- Changed password to 123456789

### Pins
- MotorRight : 13(PWM),6,5 : ENC:(17,27)
- LeftRight :  12(PWM),7,1 : ENC (21,20)
- MPU>RPI - SDA>SDA(GPIO2) and SCL>SCL(GPIO3)

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
