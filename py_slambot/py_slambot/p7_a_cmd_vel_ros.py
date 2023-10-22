import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Initialize GPIO for motors
        self.init_gpio()

        # Create subscriber
        self.subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def init_gpio(self):
        self.PWM_RIGHT = 13
        self.MOTOR_RIGHT_FWD = 11
        self.MOTOR_RIGHT_REV = 5
        self.PWM_LEFT = 19
        self.MOTOR_LEFT_FWD = 6
        self.MOTOR_LEFT_REV = 26

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWM_RIGHT, GPIO.OUT)
        GPIO.setup(self.MOTOR_RIGHT_FWD, GPIO.OUT)
        GPIO.setup(self.MOTOR_RIGHT_REV, GPIO.OUT)
        GPIO.setup(self.PWM_LEFT, GPIO.OUT)
        GPIO.setup(self.MOTOR_LEFT_FWD, GPIO.OUT)
        GPIO.setup(self.MOTOR_LEFT_REV, GPIO.OUT)

        self.pwm_right = GPIO.PWM(self.PWM_RIGHT, 100)
        self.pwm_left = GPIO.PWM(self.PWM_LEFT, 100)

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Here, make necessary calculations to convert linear and angular velocity
        # to right and left wheel speeds. The following is a simple example:
        right_speed = linear_vel + angular_vel
        left_speed = linear_vel - angular_vel
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))


        # Convert speeds to percentages for PWM. This step depends on your robot's specifics.
        right_speed_percentage = right_speed * 100  # Example conversion
        left_speed_percentage = left_speed * 100

        self.drive_motors(right_speed_percentage, left_speed_percentage)

    def drive_motors(self, right_speed, left_speed):
        if right_speed > 0:
            GPIO.output(self.MOTOR_RIGHT_FWD, GPIO.HIGH)
            self.pwm_right.start(right_speed)
        elif right_speed < 0:
            GPIO.output(self.MOTOR_RIGHT_REV, GPIO.HIGH)
            self.pwm_right.start(-right_speed)
        else:
            self.pwm_right.stop()

        if left_speed > 0:
            GPIO.output(self.MOTOR_LEFT_FWD, GPIO.HIGH)
            self.pwm_left.start(left_speed)
        elif left_speed < 0:
            GPIO.output(self.MOTOR_LEFT_REV, GPIO.HIGH)
            self.pwm_left.start(-left_speed)
        else:
            self.pwm_left.stop()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
