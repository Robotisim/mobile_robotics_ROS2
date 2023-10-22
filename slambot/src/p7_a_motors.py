import RPi.GPIO as GPIO
import time

# Define GPIO pin assignments for right and left motors
PWM_RIGHT = 13
MOTOR_RIGHT_FWD = 11
MOTOR_RIGHT_REV = 5

PWM_LEFT = 19
MOTOR_LEFT_FWD = 6
MOTOR_LEFT_REV = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_RIGHT, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FWD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_REV, GPIO.OUT)
GPIO.setup(PWM_LEFT, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_FWD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_REV, GPIO.OUT)

pwm_right = GPIO.PWM(PWM_RIGHT, 100)  # Initialize PWM for right motor at 100Hz
pwm_left = GPIO.PWM(PWM_LEFT, 100)  # Initialize PWM for left motor at 100Hz

def drive_forward(speed_percentage):
    GPIO.output(MOTOR_RIGHT_FWD, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_FWD, GPIO.HIGH)
    pwm_right.start(speed_percentage)
    pwm_left.start(speed_percentage)

def drive_backward(speed_percentage):
    GPIO.output(MOTOR_RIGHT_REV, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_REV, GPIO.HIGH)
    pwm_right.start(speed_percentage)
    pwm_left.start(speed_percentage)

def stop_motors():
    pwm_right.stop()
    pwm_left.stop()
    GPIO.output(MOTOR_RIGHT_FWD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_REV, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_FWD, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_REV, GPIO.LOW)

# Drive both motors forward at 80% speed for 3 seconds
print("Driving motors forward")
drive_forward(80)
time.sleep(3)

# Stop both motors
print("Stopping motors")
stop_motors()
time.sleep(1)

# Drive both motors backward at 80% speed for 3 seconds
print("Driving motors backward")
drive_backward(80)
time.sleep(3)

# Stop both motors and cleanup
print("Stopping motors")
stop_motors()
GPIO.cleanup()
