### Installations
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
