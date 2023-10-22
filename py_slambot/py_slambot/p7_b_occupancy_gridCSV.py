import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import csv

import os

class LidarMapping(Node):
    def __init__(self):
        super().__init__('lidar_mapping')

        # Parameters for occupancy grid (change as needed)
        self.map_width = 100  # in cells
        self.map_height = 100
        self.resolution = 0.1  # cell size in meters

        # Create publisher and subscriber
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        # Create an initial empty map
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"  # Set the frame ID
        self.map.info.width = self.map_width
        self.map.info.height = self.map_height
        self.map.info.resolution = self.resolution
        self.map.data = [0] * (self.map_width * self.map_height)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.send_transform()


        # Create CSV file with headers if it doesn't exist
        self.csv_filename = "lidar_and_grid_data.csv"
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, 'w') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(["lidar_x", "lidar_y", "grid_cell_x", "grid_cell_y"])



    def send_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "lidar_frame"
        t.transform.translation.x = 0.0  # Modify as per your needs
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0  # Quaternion representing no rotation
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)

    def lidar_callback(self, msg):
        # Process the LIDAR scan data and populate the occupancy grid
        # This is a basic example and may need more logic based on LIDAR characteristics
        with open(self.csv_filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile)
            for index, range_value in enumerate(msg.ranges):
                if range_value < msg.range_max:
                    theta = msg.angle_min + index * msg.angle_increment
                    x = range_value * np.cos(theta)
                    y = range_value * np.sin(theta)

                    # Convert the LIDAR point to a map cell index
                    # (You may need more complex logic based on your LIDAR's position and orientation)
                    cell_x = int(x / self.resolution)
                    cell_y = int(y / self.resolution)

                    # Set the cell as occupied
                    self.map.data[cell_y * self.map_width + cell_x] = 100
                    #Write to CSV
                    csvwriter.writerow([x, y, cell_x, cell_y])
        self.map.header.stamp = self.get_clock().now().to_msg()
        # Publish the updated map
        self.publisher_.publish(self.map)


def main(args=None):
    rclpy.init(args=args)
    node = LidarMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
