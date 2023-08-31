#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

// Define GPIO pin assignments for encoders
const int ENCODER_RIGHT_A = 20;
const int ENCODER_RIGHT_B = 12;
const int ENCODER_LEFT_A = 21;
const int ENCODER_LEFT_B = 16;

int encoder_count_right = 0;
int encoder_count_left = 0;

// Callback function to handle right encoder interrupts
void encoder_callback_right(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    if (gpio_read(pi, ENCODER_RIGHT_A) == gpio_read(pi, ENCODER_RIGHT_B)) {
        encoder_count_right--;
    } else {
        encoder_count_right++;
    }
}

// Callback function to handle left encoder interrupts
void encoder_callback_left(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    if (gpio_read(pi, ENCODER_LEFT_A) == gpio_read(pi, ENCODER_LEFT_B)) {
        encoder_count_left--;
    } else {
        encoder_count_left++;
    }
}

int main() {
    char *addrStr = NULL;
    char *portStr = NULL;
    int pi = pigpio_start(addrStr, portStr);

    // Set up encoder pins as inputs
    set_mode(pi, ENCODER_RIGHT_A, PI_INPUT);
    set_mode(pi, ENCODER_RIGHT_B, PI_INPUT);
    set_mode(pi, ENCODER_LEFT_A, PI_INPUT);
    set_mode(pi, ENCODER_LEFT_B, PI_INPUT);

    // Set up callbacks for encoder interrupts
    callback(pi, ENCODER_RIGHT_A, EITHER_EDGE, encoder_callback_right);
    callback(pi, ENCODER_LEFT_A, EITHER_EDGE, encoder_callback_left);

    // Monitor and print the encoder counts
    while (true) {
        std::cout << "Right Encoder Count: " << encoder_count_right << ", ";
        std::cout << "Left Encoder Count: " << encoder_count_left << std::endl;
        usleep(100000); // Sleep for 100 milliseconds
    }

    // Clean up
    pigpio_stop(pi);
    return 0;
}
