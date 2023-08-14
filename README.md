# ROS2 Mobile Robotics
This repository will be carrying code for 2D Slam ros2 based Robot in CPP .

## Setup RPI for ROS2 Development
- Install RPI Imager → Ubuntu 22.04 Server
    - Setup Wifi network file change ssid and password → no need of HDMI display
- Vscode SSH → find ip using network tools android app
- SSH username : ubuntu
- SSH Password : ubuntu
- Changed password to 123456789
- Install pigpio (v79)
    ```
    sudo apt-get install unzip && sudo apt-get update
    wget https://github.com/joan2937/pigpio/archive/v74.zip
    unzip v79.zip
    cd pigpio-79
    make
    sudo make install
    ```
- Test Library in the script directory
    ```
    g++ -o test_ blink_test.cpp -lpigpiod_if2 -lrt
    ```
- installing Rpildar Package
    ```
    sudo apt-get install ros-humble-rplidar-ros
    ```
### Pins
- MotorRight : 13(PWM),6,5 : ENC:(17,27)
- LeftRight :  12(PWM),7,1 : ENC (21,20)


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
