import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from .WaypointHandler import WaypointHandler
from .PID import PID
from .PurePursuit import PurePursuit
import math
import numpy as np

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.car_length = 0.3302
        self.current_steering = 0.0
        self.current_speed = 0.0
        self.boost = 0.0

        self.waypoint_handler = WaypointHandler()
        self.pure_pursuit = PurePursuit()
        self.angle_pid = PID(kp=1.0, ki=0.0, kd=0.0)
        self.speed_pid = PID(kp=1.0, ki=0.0, kd=0.0)

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

    def get_drive(self, car_odom, scan):
        car_pos = (car_odom.pose.pose.position.x, car_odom.pose.pose.position.y)
        car_quat = car_odom.pose.pose.orientation
        car_heading_world = euler_from_quaternion((car_quat.x, car_quat.y, car_quat.z, car_quat.w))[2]

        # Get next waypoint, angle, and distance
        k_dd = 0.15
        lookahead = self.current_speed * k_dd + self.car_length
        lookahead = max(lookahead, 2.0)
        use_direction = False  # PARAMETER, False works well for fast speeds
        next_waypoint, angle_to_waypoint_world, waypoint_angle, distance_to_waypoint = self.waypoint_handler.get_next_waypoint(car_pos, car_heading_world, lookahead, use_direction)

        if next_waypoint is None:
            return (0.0, 0.0)  # stop until we get waypoints

        # Publish a marker at the next waypoint
        self.publish_marker(next_waypoint.pose.position)

        alpha = (car_heading_world - angle_to_waypoint_world) % (2 * math.pi)

        # Pure Pursuit
        angle_adjustment = self.pure_pursuit.get_control(self.car_length, alpha, distance_to_waypoint)
        angle_adjustment *= 1.35  # helps account for kinematic vs dynamic model
        angle_adjustment = self.angle_pid.update(angle_adjustment)

        steering = max(min(angle_adjustment, 0.6), -0.6)
        self.current_steering = steering

        if next_waypoint.pose.position.z > 0.0:
            speed_error = 1.1 * next_waypoint.pose.position.z - (self.current_speed - self.boost)
        else:  # probably centerline
            speed_error = 3.0 - (self.current_speed - self.boost)

        self.current_speed += self.speed_pid.update(speed_error)

        return (self.current_speed, steering)

    def publish_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
