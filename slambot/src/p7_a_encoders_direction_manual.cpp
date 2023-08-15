#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

const int ENCODER_RIGHT_A = 27; //21
const int ENCODER_RIGHT_B = 17; //20

int encoder_count_right = 0;

// Callback function to handle right encoder interrupts
void encoder_callback_right(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    if (gpio_read(pi, ENCODER_RIGHT_A) == gpio_read(pi, ENCODER_RIGHT_B)) {
        encoder_count_right--;
    } else {
        encoder_count_right++;
    }
}

int main() {
    char *addrStr = NULL;
    char *portStr = NULL;
    int pi = pigpio_start(addrStr, portStr);

    // Set up encoder pins as inputs
    set_mode(pi, ENCODER_RIGHT_A, PI_INPUT);
    set_mode(pi, ENCODER_RIGHT_B, PI_INPUT);

    // Set up callbacks for encoder interrupts
    callback(pi, ENCODER_RIGHT_A, EITHER_EDGE, encoder_callback_right);

    // Monitor and print the encoder count
    while (true) {
        std::cout << "Right Encoder Count: " << encoder_count_right << std::endl;
        usleep(100000); // Sleep for 100 milliseconds
    }

    // Clean up
    pigpio_stop(pi);
    return 0;
}
