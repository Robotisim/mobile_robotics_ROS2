#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

// Define GPIO pin assignments for encoders
const int ENCODER_RIGHT_A = 27;
const int ENCODER_RIGHT_B = 17;
const int ENCODER_LEFT_A = 21;
const int ENCODER_LEFT_B = 20;

int encoder_count_right = 0;
int encoder_count_left = 0;
int last_AB_right = 0b00;
int last_AB_left = 0b00;
const int8_t ENC_STATES[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Callback function to handle right encoder interrupts
void encoder_callback_right(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    int A = gpio_read(pi, ENCODER_RIGHT_A);
    int B = gpio_read(pi, ENCODER_RIGHT_B);
    int current_AB = (A << 1) | B;
    int position = (last_AB_right << 2) | current_AB;
    encoder_count_right += ENC_STATES[position];
    last_AB_right = current_AB;
}

// Callback function to handle left encoder interrupts
void encoder_callback_left(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    int A = gpio_read(pi, ENCODER_LEFT_A);
    int B = gpio_read(pi, ENCODER_LEFT_B);
    int current_AB = (A << 1) | B;
    int position = (last_AB_left << 2) | current_AB;
    encoder_count_left += ENC_STATES[position];
    last_AB_left = current_AB;
}

int main() {
    char *addrStr = NULL;
    char *portStr = NULL;
    int pi = pigpio_start(addrStr, portStr);

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

    // Print encoder counts in an infinite loop
    while (true) {
        std::cout << "Right Encoder: " << encoder_count_right
                  << ", Left Encoder: " << encoder_count_left << std::endl;
        usleep(500000); // Half-second delay for readability
    }

    // Clean up
    pigpio_stop(pi);
    return 0;
}
