#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

def callback(msg):
    # Convert the CompressedImage message to an OpenCV image
    bridge = CvBridge()
    image = bridge.compressed_imgmsg_to_cv2(msg)

    # Display the image (you can use any GUI framework you prefer)
    cv2.imshow("ESP32-CAM Video Stream", image)
    cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('esp32_cam_viewer')
    subscriber = node.create_subscription(
        CompressedImage,
        'video_stream',  # Replace with the correct topic name
        callback,
        10)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
