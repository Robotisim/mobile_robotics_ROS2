# #!/usr/bin/env python

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Quaternion
# from nav_msgs.msg import Odometry
# from tf2_ros.transform_broadcaster import TransformBroadcaster
# from std_msgs.msg import Int32
# from math import sin, cos, pi
# import tf2_ros
# from geometry_msgs.msg import TransformStamped

# class DiffTf(Node):

#     def __init__(self):
#         super().__init__('diff_tf')
#         self.get_logger().info("-I- %s started" % self.get_name())

#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('rate', 10.0),
#                 ('ticks_meter', 50.0),
#                 ('base_width', 0.245),
#                 ('base_frame_id', 'base_link'),
#                 ('odom_frame_id', 'odom'),
#                 ('encoder_min', -32768),
#                 ('encoder_max', 32768),
#                 ('wheel_low_wrap', 0.3),
#                 ('wheel_high_wrap', 0.7)
#             ]
#         )

#         self.rate = self.get_parameter('rate').value
#         self.ticks_meter = float(self.get_parameter('ticks_meter').value)
#         self.base_width = float(self.get_parameter('base_width').value)
#         self.base_frame_id = self.get_parameter('base_frame_id').value
#         self.odom_frame_id = self.get_parameter('odom_frame_id').value
#         self.encoder_min = self.get_parameter('encoder_min').value
#         self.encoder_max = self.get_parameter('encoder_max').value
#         self.encoder_low_wrap = self.get_parameter('wheel_low_wrap').value * (self.encoder_max - self.encoder_min) + self.encoder_min
#         self.encoder_high_wrap = self.get_parameter('wheel_high_wrap').value * (self.encoder_max - self.encoder_min) + self.encoder_min

#         self.t_delta = rclpy.duration.Duration(seconds=1.0 / self.rate)
#         self.t_next = self.get_clock().now() + self.t_delta

#         self.enc_left = None
#         self.enc_right = None
#         self.left = 0
#         self.right = 0
#         self.lmult = 0
#         self.rmult = 0
#         self.prev_lencoder = 0
#         self.prev_rencoder = 0
#         self.x = 0
#         self.y = 0
#         self.th = 0
#         self.dx = 0
#         self.dr = 0
#         self.then = self.get_clock().now()

#         self.create_subscription(Int32, '/encoder0_topic', self.lwheelCallback, 10)
#         self.create_subscription(Int32, '/encoder1_topic', self.rwheelCallback, 10)
#         self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
#         self.odom_broadcaster = TransformBroadcaster(self)

#     def spin(self):
#         rate = self.create_rate(self.rate)
#         while rclpy.ok():
#             self.update()
#             rate.sleep()

#     def update(self):
#         now = self.get_clock().now()
#         if now > self.t_next:
#             elapsed = now - self.then
#             self.then = now
#             elapsed = elapsed.to_sec()

#             if self.enc_left is None:
#                 d_left = 0
#                 d_right = 0
#             else:
#                 d_left = (self.left - self.enc_left) / self.ticks_meter
#                 d_right = (self.right - self.enc_right) / self.ticks_meter
#             self.enc_left = self.left
#             self.enc_right = self.right

#             d = (d_left + d_right) / 2
#             th = (d_right - d_left) / self.base_width
#             self.dx = d / elapsed
#             self.dr = th / elapsed

#             if d != 0:
#                 x = cos(th) * d
#                 y = -sin(th) * d
#                 self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
#                 self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
#             if th != 0:
#                 self.th = self.th + th


#             quaternion = tf2_ros.transformations.quaternion_from_euler(0, 0, self.th)
#             self.odom_broadcaster.sendTransform(
#                 (self.x, self.y, 0),
#                 quaternion,
#                 now,
#                 self.base_frame_id,
#                 self.odom_frame_id
#             )

#             odom = Odometry()
#             odom.header.stamp = now
#             odom.header.frame_id = self.odom_frame_id
#             odom.pose.pose.position.x = self.x
#             odom.pose.pose.position.y = self.y
#             odom.pose.pose.position.z = 0
#             odom.pose.pose.orientation = Quaternion(*quaternion)
#             odom.child_frame_id = self.base_frame_id
#             odom.twist.twist.linear.x = self.dx
#             odom.twist.twist.linear.y = 0
#             odom.twist.twist.angular.z = self.dr
#             self.odom_pub.publish(odom)

#     def lwheelCallback(self, msg):
#         enc = msg.data
#         if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
#             self.lmult = self.lmult + 1

#         if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
#             self.lmult = self.lmult - 1

#         self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
#         self.prev_lencoder = enc

#     def rwheelCallback(self, msg):
#         enc = msg.data
#         if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
#             self.rmult = self.rmult + 1

#         if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
#             self.rmult = self.rmult - 1

#         self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
#         self.prev_rencoder = enc


# def main(args=None):
#     rclpy.init(args=args)
#     diff_tf = DiffTf()
#     diff_tf.spin()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
from math import sin, cos, pi
import tf2_ros
from geometry_msgs.msg import TransformStamped

class DiffTf(Node):

    def __init__(self):
        super().__init__('diff_tf')
        self.get_logger().info("-I- %s started" % self.get_name())

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rate', 10.0),
                ('ticks_meter', 50.0),
                ('base_width', 0.245),
                ('base_frame_id', 'base_link'),
                ('odom_frame_id', 'odom'),
                ('encoder_min', -100000),
                ('encoder_max', 100000),
                ('wheel_low_wrap', 0.3),
                ('wheel_high_wrap', 0.7)
            ]
        )

        self.rate = self.get_parameter('rate').value
        self.ticks_meter = float(self.get_parameter('ticks_meter').value)
        self.base_width = float(self.get_parameter('base_width').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.encoder_min = self.get_parameter('encoder_min').value
        self.encoder_max = self.get_parameter('encoder_max').value
        self.encoder_low_wrap = self.get_parameter('wheel_low_wrap').value * (self.encoder_max - self.encoder_min) + self.encoder_min
        self.encoder_high_wrap = self.get_parameter('wheel_high_wrap').value * (self.encoder_max - self.encoder_min) + self.encoder_min

        self.t_delta = rclpy.duration.Duration(seconds=1.0 / self.rate)
        self.t_next = self.get_clock().now() + self.t_delta

        self.enc_left = None
        self.enc_right = None
        self.left = 0
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0
        self.then = self.get_clock().now()

        self.create_subscription(Int32, '/encoder0_topic', self.lwheelCallback, 10)
        self.create_subscription(Int32, '/encoder1_topic', self.rwheelCallback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_broadcaster = TransformBroadcaster(self)

    def spin(self):
        rate = self.create_rate(self.rate)
        while rclpy.ok():
            self.update()
            rate.sleep()

    def update(self):
        now = self.get_clock().now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if self.enc_left is None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right

            d = (d_left + d_right) / 2
            th = (d_right - d_left) / self.base_width
            self.dx = d / elapsed
            self.dr = th / elapsed

            if d != 0:
                x = cos(th) * d
                y = -sin(th) * d
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            if th != 0:
                self.th = self.th + th

            quaternion = tf2_ros.transformations.quaternion_from_euler(0, 0, self.th)
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0),
                quaternion,
                now,
                self.base_frame_id,
                self.odom_frame_id
            )

            # Add the following lines
            quat = tf2_ros.transformations.quaternion_from_euler(0, 0, self.th)
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.base_frame_id
            tf_msg.child_frame_id = self.odom_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = quat[0]
            tf_msg.transform.rotation.y = quat[1]
            tf_msg.transform.rotation.z = quat[2]
            tf_msg.transform.rotation.w = quat[3]
            self.odom_broadcaster.sendTransform(tf_msg)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = Quaternion(*quaternion)
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odom_pub.publish(odom)

    def lwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

    def rwheelCallback(self, msg):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc


def main(args=None):
    rclpy.init(args=args)
    diff_tf = DiffTf()
    diff_tf.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()