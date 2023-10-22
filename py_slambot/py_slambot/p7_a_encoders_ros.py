import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import RPi.GPIO as GPIO
import time

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')

        # Initialize GPIO for encoders
        self.init_gpio()

        # Create publishers
        self.left_pub = self.create_publisher(Int16, 'left_enc', 10)
        self.right_pub = self.create_publisher(Int16, 'right_enc', 10)

        # Timer to periodically read and publish encoder values
        self.timer = self.create_timer(0.0001, self.publish_encoders)

    def init_gpio(self):
        self.left_A_pin = 21
        self.left_B_pin = 16
        self.right_A_pin = 12
        self.right_B_pin = 20

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_A_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.left_B_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_A_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_B_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.outcome = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]
        self.left_last_AB = 0b00
        self.right_last_AB = 0b00
        self.left_counter = 0
        self.right_counter = 0

    def read_encoder(self, A_pin, B_pin, last_AB, counter):
        A = GPIO.input(A_pin)
        B = GPIO.input(B_pin)
        current_AB = (A << 1) | B
        position = (last_AB << 2) | current_AB
        counter += self.outcome[position]
        last_AB = current_AB
        # time.sleep(0.002)  # debounce delay
        return last_AB, counter

    def publish_encoders(self):
        start_time = time.time()

        self.left_last_AB, self.left_counter = self.read_encoder(self.left_A_pin, self.left_B_pin, self.left_last_AB, self.left_counter)
        self.right_last_AB, self.right_counter = self.read_encoder(self.right_A_pin, self.right_B_pin, self.right_last_AB, self.right_counter)

        left_msg = Int16()
        right_msg = Int16()

        left_msg.data = self.left_counter
        right_msg.data = self.right_counter

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        end_time = time.time()
        elapsed_time = end_time - start_time

        # Print the encoder readings and the time taken to publish
        print(f"Encoder (Left/Right): {self.left_counter} / {self.right_counter}. Time taken: {elapsed_time:.6f} seconds")




def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Node Started")
    main()

