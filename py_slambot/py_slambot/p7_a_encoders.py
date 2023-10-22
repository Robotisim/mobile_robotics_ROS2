import RPi.GPIO as GPIO
import time

left_A_pin = 21
left_B_pin = 16
right_A_pin = 12
right_B_pin = 20

GPIO.setmode(GPIO.BCM)

# Setup pins for both left and right encoders
GPIO.setup(left_A_pin, GPIO.IN)
GPIO.setup(left_B_pin, GPIO.IN)
GPIO.setup(right_A_pin, GPIO.IN)
GPIO.setup(right_B_pin, GPIO.IN)

outcome = [0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]
left_last_AB = 0b00
right_last_AB = 0b00
left_counter = 0
right_counter = 0

try:
    while True:
        start_time = time.time()  # Start time

        # Read left encoder
        left_A = GPIO.input(left_A_pin)
        left_B = GPIO.input(left_B_pin)
        left_current_AB = (left_A << 1) | left_B
        left_position = (left_last_AB << 2) | left_current_AB
        left_counter = left_counter - outcome[left_position]
        left_last_AB = left_current_AB

        # Read right encoder
        right_A = GPIO.input(right_A_pin)
        right_B = GPIO.input(right_B_pin)
        right_current_AB = (right_A << 1) | right_B
        right_position = (right_last_AB << 2) | right_current_AB
        right_counter = right_counter - outcome[right_position]
        right_last_AB = right_current_AB

        end_time = time.time()  # End time

        elapsed_time = end_time - start_time
        print(f"Encoder (Left/Right): {left_counter} / {right_counter}. Time taken: {elapsed_time:.6f} seconds")

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
