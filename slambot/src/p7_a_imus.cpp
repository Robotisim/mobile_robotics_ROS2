#include <iostream>
#include <pigpiod_if2.h>
#include "SPI_MPU9250.h"

int main() {
    int pi = pigpio_start(NULL, NULL); // Connect to pigpio daemon
    if (pi < 0) {
        std::cerr << "Failed to connect to pigpio. Is the daemon running?" << std::endl;
        return -1;
    }

    SPI_MPU9250 imu(pi, 0); // Initialize the MPU-9250 on SPI channel 0

    if (!imu.begin()) {
        std::cerr << "Failed to initialize MPU-9250" << std::endl;
        return -1;
    }

    while (true) {
        imu.update(); // Update the sensor data

        // Log the orientation data
        std::cout << "Orientation - Roll: " << imu.getRoll()
                  << ", Pitch: " << imu.getPitch()
                  << ", Yaw: " << imu.getYaw() << std::endl;

        time_sleep(0.1); // Sleep for 100 milliseconds
    }

    pigpio_stop(pi); // Disconnect from pigpio
    return 0;
}
