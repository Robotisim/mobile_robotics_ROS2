#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_msgs.msg import Int16
import tf_transformations
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.encoder_left_subscription = self.create_subscription(Int16, 'left_enc', self.left_encoder_callback, 10)
        self.encoder_right_subscription = self.create_subscription(Int16, 'right_enc', self.right_encoder_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.wheel_separation = 0.165  # Wheel separation in meters
        self.wheel_radius = 0.0675  # Wheel radius in meters
        self.ticks_per_revolution = 380
        self.distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution

    def left_encoder_callback(self, msg):
        current_ticks = msg.data
        delta_ticks = current_ticks - self.last_left_ticks
        self.last_left_ticks = current_ticks
        distance = delta_ticks * self.distance_per_tick
        self.update_position(distance, 0)

    def right_encoder_callback(self, msg):
        current_ticks = msg.data
        delta_ticks = current_ticks - self.last_right_ticks
        self.last_right_ticks = current_ticks
        distance = delta_ticks * self.distance_per_tick
        self.update_position(0, distance)

    def update_position(self, left_distance, right_distance):
        distance = (left_distance + right_distance) / 2
        angle = (right_distance - left_distance) / self.wheel_separation

        if angle != 0:
            radius = distance / angle
            self.x += radius * (math.sin(angle + self.th) - math.sin(self.th))
            self.y -= radius * (math.cos(angle + self.th) - math.cos(self.th))
            self.th += angle
        else:
            self.x += distance * math.cos(self.th)
            self.y += distance * math.sin(self.th)

        self.publish_odometry()

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom_msg.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Optional: Add velocity calculation here

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # Assuming a flat surface
        t.transform.rotation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Broadcast the dynamic transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
