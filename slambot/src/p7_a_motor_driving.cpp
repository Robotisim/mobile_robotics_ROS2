#include <iostream>
#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>

// Define GPIO pin assignments for right and left motors
const int PWM_RIGHT = 13;
const int MOTOR_RIGHT_FWD = 6;
const int MOTOR_RIGHT_REV = 5;

const int PWM_LEFT = 12;
const int MOTOR_LEFT_FWD = 16;
const int MOTOR_LEFT_REV = 26;

int pigpio_setup() {
    char *addrStr = NULL;
    char *portStr = NULL;
    const int pi = pigpio_start(addrStr, portStr);

    // Set up pins for right motor
    set_mode(pi, PWM_RIGHT, PI_OUTPUT);
    set_mode(pi, MOTOR_RIGHT_FWD, PI_OUTPUT);
    set_mode(pi, MOTOR_RIGHT_REV, PI_OUTPUT);

    // Set up pins for left motor
    set_mode(pi, PWM_LEFT, PI_OUTPUT);
    set_mode(pi, MOTOR_LEFT_FWD, PI_OUTPUT);
    set_mode(pi, MOTOR_LEFT_REV, PI_OUTPUT);

    // Initialize motors off
    gpio_write(pi, MOTOR_RIGHT_FWD, 1);
    gpio_write(pi, MOTOR_RIGHT_REV, 1);
    gpio_write(pi, MOTOR_LEFT_FWD, 1);
    gpio_write(pi, MOTOR_LEFT_REV, 1);

    return pi;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("motor_control");

    int pi = pigpio_setup();
    if (pi < 0) {
        RCLCPP_ERROR(logger, "Failed to connect to Pigpio Daemon. Is it running?");
        return -1;
    }

    // Drive both motors forward at speed 200 for 3 seconds
    RCLCPP_INFO(logger, "Driving motors forward");
    gpio_write(pi, MOTOR_RIGHT_FWD, 0);
    gpio_write(pi, MOTOR_LEFT_FWD, 0);
    set_PWM_dutycycle(pi, PWM_RIGHT, 200);
    set_PWM_dutycycle(pi, PWM_LEFT, 200);
    time_sleep(3);

    // Stop both motors
    RCLCPP_INFO(logger, "Stopping motors");
    gpio_write(pi, MOTOR_RIGHT_FWD, 1);
    gpio_write(pi, MOTOR_LEFT_FWD, 1);
    time_sleep(1);

    // Drive both motors backward at speed 200 for 3 seconds
    RCLCPP_INFO(logger, "Driving motors backward");
    gpio_write(pi, MOTOR_RIGHT_REV, 0);
    gpio_write(pi, MOTOR_LEFT_REV, 0);
    set_PWM_dutycycle(pi, PWM_RIGHT, 200);
    set_PWM_dutycycle(pi, PWM_LEFT, 200);
    time_sleep(3);

    // Stop both motors
    RCLCPP_INFO(logger, "Stopping motors");
    gpio_write(pi, MOTOR_RIGHT_REV, 1);
    gpio_write(pi, MOTOR_LEFT_REV, 1);

    pigpio_stop(pi);
    rclcpp::shutdown();
    return 0;
}
