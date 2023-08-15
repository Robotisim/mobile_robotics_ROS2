#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

// Define GPIO pin assignments for motors and encoders
const int PWM_RIGHT = 13;
const int MOTOR_RIGHT_FWD = 6;
const int MOTOR_RIGHT_REV = 5;
const int ENCODER_RIGHT_A = 27;
const int ENCODER_RIGHT_B = 17;

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

    // Number of repetitions for the test
    const int repetitions = 5;

    // Repeatability test
    for (int i = 0; i < repetitions; i++) {
        // Reset encoder counts
        encoder_count_right = 0;
        encoder_count_left = 0;

        // Determine direction based on repetition number (odd or even)
        bool isOddRepetition = i % 2 != 0;

        // Set motor direction
        gpio_write(pi, MOTOR_RIGHT_FWD, isOddRepetition);
        gpio_write(pi, MOTOR_LEFT_FWD, isOddRepetition);
        gpio_write(pi, MOTOR_RIGHT_REV, !isOddRepetition);
        gpio_write(pi, MOTOR_LEFT_REV, !isOddRepetition);

        // Set PWM duty cycle
        set_PWM_dutycycle(pi, PWM_RIGHT, 200);
        set_PWM_dutycycle(pi, PWM_LEFT, 230);

        // Move for 2 seconds
        sleep(2);

        // Stop motors
        gpio_write(pi, MOTOR_RIGHT_FWD, 1);
        gpio_write(pi, MOTOR_LEFT_FWD, 1);
        gpio_write(pi, MOTOR_RIGHT_REV, 1);
        gpio_write(pi, MOTOR_LEFT_REV, 1);

        // Log encoder ticks
        std::cout << "Repetition " << i + 1 << " - Right Encoder: " << encoder_count_right
                  << ", Left Encoder: " << encoder_count_left << std::endl;
    }

    // Clean up
    pigpio_stop(pi);
    return 0;
}
