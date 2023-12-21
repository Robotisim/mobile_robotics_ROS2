#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import serial
import threading
import sys

class SerialEncoderReader(Node):
    def __init__(self):
        super().__init__('serial_encoder_reader')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.left_encoder_publisher = self.create_publisher(Int16, 'left_enc', 10)
        self.right_encoder_publisher = self.create_publisher(Int16, 'right_enc', 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.serial_thread = threading.Thread(target=self.read_from_serial, args=(serial_port, baud_rate))
        self.serial_thread.daemon = True
        self.serial_thread.start()

        try:
            self.serial_port = serial.Serial(serial_port, baud_rate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {e}')
            sys.exit(1)

    def cmd_vel_callback(self, msg):
        # Calculate PWM values based on cmd_vel (msg.linear.x, msg.angular.z)
        # These calculations depend on your specific robot's kinematics
        left_pwm = int(msg.linear.x - msg.angular.z)
        right_pwm = int(msg.linear.x + msg.angular.z)

        # Send PWM values to ESP32 via serial
        self.serial_port.write(f'{left_pwm},{right_pwm}\n'.encode('utf-8'))
        self.get_logger().error(f'Error opening serial port: {e}')

    def read_from_serial(self, port, baud):
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                while rclpy.ok():
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        left_val, right_val = map(int, line.split(','))
                        self.left_encoder_publisher.publish(Int16(data=left_val))
                        self.right_encoder_publisher.publish(Int16(data=right_val))
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {e}')
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = SerialEncoderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
