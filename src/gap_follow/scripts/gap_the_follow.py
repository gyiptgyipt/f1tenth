#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/drive', 10)
        self.subscription

    def preprocess_lidar(self, ranges):
        proc_ranges = np.array(ranges)
        proc_ranges = np.where(np.isnan(proc_ranges), np.inf, proc_ranges)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        contiguous_chunks = np.ma.clump_unmasked(masked)
        max_gap = max(contiguous_chunks, key=lambda x: x.stop - x.start)
        return max_gap.start, max_gap.stop

    def find_best_point(self, start_i, end_i, ranges):
        return (start_i + end_i) // 2

    def lidar_callback(self, msg):
        ranges = self.preprocess_lidar(msg.ranges)
        
        # Define parameters
        bubble_radius = 20
        preprocess_conv_size = 3
        max_range = 3.0

        # Apply a convolution filter
        conv_filter = np.ones(preprocess_conv_size)/preprocess_conv_size
        ranges = np.convolve(ranges, conv_filter, 'same')

        # Eliminate all ranges further than max_range
        ranges = np.where(ranges > max_range, 0, ranges)

        # Bubble filter to eliminate obstacles
        closest = ranges.argmin()
        min_index = max(0, closest - bubble_radius)
        max_index = min(len(ranges) - 1, closest + bubble_radius)
        ranges[min_index:max_index] = 0

        # Find the largest gap
        start_i, end_i = self.find_max_gap(ranges)

        # Find the best point in the gap
        best_point = self.find_best_point(start_i, end_i, ranges)

        # Calculate the steering angle
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        steering_angle = angle_min + best_point * angle_increment

        # Publish the drive message
        drive_msg = Twist()
        drive_msg.linear.x = 1.0  # Set a constant speed
        drive_msg.angular.z = steering_angle
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    follow_the_gap = FollowTheGap()
    rclpy.spin(follow_the_gap)
    follow_the_gap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
