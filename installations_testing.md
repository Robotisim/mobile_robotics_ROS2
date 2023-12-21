### Installations
- User Dialout
    ```
    sudo usermod -a -G dialout $USER
    sudo reboot
    ```
- Install ros2 base
- ROS2 packages
    ```
    sudo apt-get install ros-humble-teleop-twist-keyboard
    ```
- Lidar Sensor
    ```
    sudo apt-get install ros-humble-rplidar-ros
    sudo apt-get install ros-humble-slam-toolbox
    sudo apt-get install ros-humble-cartographer-ros

    ```
- Install pigpio (v79)
    ```
    sudo apt-get install unzip && sudo apt-get update
    wget https://github.com/joan2937/pigpio/archive/v74.zip
    unzip v79.zip
    cd pigpio-79
    make
    sudo make install
    ```
- Rpildar Package
    ```
    sudo apt-get install ros-humble-rplidar-ros

    ```
- I2C library testing -> [Complete Guide](https://devicetests.com/enabling-i2c-raspberry-pi-ubuntu)
    ```
    sudo apt-get install -y i2c-tools
    sudo apt update
    sudo apt install g++

    ```
- Serial Com
    ```
    pip3 install pyserial
    ```
- Transforms for Odom
    ```
    sudo apt install ros-humble-tf-transformations
    sudo pip3 install transforms3d

    ```
----------
### Running
- ros2 launch slambot e_slam
- MicroROS Installation Setup (USB)
    ```
    https://github.com/Robotisim/mobile_robotics_ROS2/wiki/Project-%234:-MicroROS-Controlled-Differential-Drive-Bot
    ```
### ON PC
    ```
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    ```
-----
### Testing Libraries - Sensors
### Pigpiod Library
- Test Library in the script directory
    ```
    sudo pigpiod
    g++ -o test_ blink_test.cpp -lpigpiod_if2 -lrt
    ```
### IMU library
- Detect i2C connection
    ```
    ls /dev/*i2c*
    ```
- Read data
    ```
    sudo i2cget 1 0x68 0x01
    ```
- Read data again
    ```
    sudo i2cdetect -y 1
    ```

### Errors
- RPI lidar [rplidar_composition-1] *** buffer overflow detected ***: terminated
- Reson is usb cable not data connection -> ls dev/ttyU* -> no output
 ```
 sudo chmod 777 /dev/ttyUSB0
 ```
