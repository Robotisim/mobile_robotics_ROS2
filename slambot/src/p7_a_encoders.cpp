#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

// Define GPIO pin assignments for motors and encoders
const int PWM_RIGHT = 13;
const int MOTOR_RIGHT_FWD = 6;
const int MOTOR_RIGHT_REV = 5;
const int ENCODER_RIGHT_A = 17;
const int ENCODER_RIGHT_B = 27;

const int PWM_LEFT = 12;
const int MOTOR_LEFT_FWD = 16;
const int MOTOR_LEFT_REV = 26;
const int ENCODER_LEFT_A = 21;
const int ENCODER_LEFT_B = 20;

int encoder_count_right = 0;
int encoder_count_left = 0;

// Callback function to handle right encoder interrupts
void encoder_callback_right(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    encoder_count_right++;
}

// Callback function to handle left encoder interrupts
void encoder_callback_left(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    encoder_count_left++;
}

int main() {
    char *addrStr = NULL;
    char *portStr = NULL;
    int pi = pigpio_start(addrStr, portStr);

    // Set up motor pins
    set_mode(pi, PWM_RIGHT, PI_OUTPUT);
    set_mode(pi, MOTOR_RIGHT_FWD, PI_OUTPUT);
    set_mode(pi, MOTOR_RIGHT_REV, PI_OUTPUT);
    set_mode(pi, PWM_LEFT, PI_OUTPUT);
    set_mode(pi, MOTOR_LEFT_FWD, PI_OUTPUT);
    set_mode(pi, MOTOR_LEFT_REV, PI_OUTPUT);

    // Set up encoder pins as inputs with pull-up resistors
    set_mode(pi, ENCODER_RIGHT_A, PI_INPUT);
    set_mode(pi, ENCODER_RIGHT_B, PI_INPUT);
    set_mode(pi, ENCODER_LEFT_A, PI_INPUT);
    set_mode(pi, ENCODER_LEFT_B, PI_INPUT);

    // Set up callbacks for encoder interrupts
    callback(pi, ENCODER_RIGHT_A, EITHER_EDGE, encoder_callback_right);
    callback(pi, ENCODER_RIGHT_B, EITHER_EDGE, encoder_callback_right);
    callback(pi, ENCODER_LEFT_A, EITHER_EDGE, encoder_callback_left);
    callback(pi, ENCODER_LEFT_B, EITHER_EDGE, encoder_callback_left);
    // Drive motors forward for 1 second
    gpio_write(pi, MOTOR_RIGHT_FWD, 0);
    gpio_write(pi, MOTOR_LEFT_FWD, 0);
    set_PWM_dutycycle(pi, PWM_RIGHT, 200);
    set_PWM_dutycycle(pi, PWM_LEFT, 200);
    sleep(1);

    // Stop motors
    gpio_write(pi, MOTOR_RIGHT_FWD, 1);
    gpio_write(pi, MOTOR_LEFT_FWD, 1);

    // Log encoder ticks
    std::cout << "Encoder ticks after forward movement - Right: " << encoder_count_right << ", Left: " << encoder_count_left << std::endl;

    // Reset encoder counts
    encoder_count_right = 0;
    encoder_count_left = 0;

    // Drive motors backward for 1 second
    gpio_write(pi, MOTOR_RIGHT_REV, 0);
    gpio_write(pi, MOTOR_LEFT_REV, 0);
    sleep(1);

    // Stop motors
    gpio_write(pi, MOTOR_RIGHT_REV, 1);
    gpio_write(pi, MOTOR_LEFT_REV, 1);

    // Log encoder ticks
    std::cout << "Encoder ticks after backward movement - Right: " << encoder_count_right << ", Left: " << encoder_count_left << std::endl;

    // Clean up
    pigpio_stop(pi);
    return 0;
}
