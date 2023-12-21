#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <chrono>
#include <thread>

// Define GPIO pin assignments for encoders
const int ENCODER_RIGHT_A = 20;
const int ENCODER_RIGHT_B = 12;

const int ENCODER_LEFT_A = 21;
const int ENCODER_LEFT_B = 16;

int encoder_count_right = 0;
int encoder_count_left = 0;

// Mutex for thread-safe access to encoder counts
pthread_mutex_t encoder_mutex = PTHREAD_MUTEX_INITIALIZER;
const int DEBOUNCE_DELAY = 5;
// Callback function to handle right encoder interrupts
void encoder_callback_right(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    // Delay for debouncing
    if (gpio_read(pi, ENCODER_RIGHT_A) == gpio_read(pi, ENCODER_RIGHT_B)) {
            encoder_count_right--;
        } else {
            encoder_count_right++;
        }
        std::cout << "Right Encoder Count: " << encoder_count_right << std::endl;
    }

// Callback function to handle left encoder interrupts
void encoder_callback_left(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    pthread_mutex_lock(&encoder_mutex);
    if (gpio_read(pi, ENCODER_LEFT_A) == gpio_read(pi, ENCODER_LEFT_B)) {
        encoder_count_left--;
    } else {
        encoder_count_left++;
    }
    std::cout << "Left Encoder Count: " << encoder_count_left << std::endl;
    pthread_mutex_unlock(&encoder_mutex);
}

int main() {
    std::cout<<"Spin the wheel"<<std::endl;
    char *addrStr = NULL;
    char *portStr = NULL;
    int pi = pigpio_start(addrStr, portStr);

    // Set up encoder pins as inputs
    set_mode(pi, ENCODER_RIGHT_A, PI_INPUT);
    set_mode(pi, ENCODER_RIGHT_B, PI_INPUT);
    set_mode(pi, ENCODER_LEFT_A, PI_INPUT);
    set_mode(pi, ENCODER_LEFT_B, PI_INPUT);

    // Set up callbacks for encoder interrupts
    callback(pi, ENCODER_RIGHT_A, RISING_EDGE, encoder_callback_right);
    callback(pi, ENCODER_LEFT_A, RISING_EDGE, encoder_callback_left);

    // Main loop can be used for other tasks or put to sleep
    while (true) {
        sleep(1); // Sleeps for 1 second. Replace with other tasks as needed.
    }

    // Clean up
    pigpio_stop(pi);
    return 0;
}
