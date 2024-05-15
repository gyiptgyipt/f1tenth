
from .WaypointHandler import WaypointHandler
from .PID import PID
from .PurePursuit import PurePursuit

import math
import numpy as np

from tf_transformations import euler_from_quaternion, quaternion_from_euler

class WaypointNavigator:
    def __init__(self):
        self.car_length = 0.3302

        self.current_steering = 0.0
        self.current_speed = 0.0
        self.boost = 0.0

        self.waypoint_handler = WaypointHandler()
        self.pure_pursuit = PurePursuit()
        self.angle_pid = PID(kp = 1.0, ki = 0.0, kd = 0.0)
        self.speed_pid = PID(kp = 1.0, ki = 0.0, kd = 0.0)

    def get_drive(self, car_odom, scan):
        """
        Returns speed and steering
        """

        car_pos = (car_odom.pose.pose.position.x, car_odom.pose.pose.position.y)
        car_quat = car_odom.pose.pose.orientation
        car_heading_world = euler_from_quaternion((car_quat.x, car_quat.y, car_quat.z, car_quat.w))[2]

        # Get next waypoint, angle, and distance
        k_dd = 0.15
        lookahead = self.current_speed * k_dd + self.car_length
        lookahead = max(lookahead, 2.0)
        use_direction = False  # PARAMETER, False works well for fast speeds
        #lookahead = -1.0 # shouldnt matter if we use direction
        next_waypoint, angle_to_waypoint_world, waypoint_angle, distance_to_waypoint = self.waypoint_handler.get_next_waypoint(car_pos, car_heading_world, lookahead, use_direction)

        if next_waypoint is None:
            return (0.0, 0.0)  # stop until we get waypoints

        alpha = (car_heading_world - angle_to_waypoint_world) % (2*math.pi)

        # Pure Pursuit
        angle_adjustment = self.pure_pursuit.get_control(self.car_length, alpha, distance_to_waypoint)
        angle_adjustment *= 1.35  # helps account for kinematic vs dynamic model
        angle_adjustment = self.angle_pid.update(angle_adjustment)

        steering = max(min(angle_adjustment, 0.6), -0.6)
        self.current_steering = steering

        if next_waypoint.pose.position.z > 0.0:
            speed_error = 1.1*next_waypoint.pose.position.z - (self.current_speed - self.boost)
        else:  # probably centerline
            speed_error = 3.0 - (self.current_speed - self.boost)

        # # lidar-based throttle boost
        # area = np.sum(scan.ranges[420:660:10]) / 24.0   # right: 350, middle: 540, left: 730
        # if area > (self.current_speed * 0.6) ** 1.1:  # the faster we go, the farther away we have to look

        #     self.boost += 0.013
        # else:
        #     if self.current_speed > 13.0:
        #         self.boost -= 0.012
        #     else:
        #         self.boost -= 0.008

        # self.current_speed += self.boost
        # self.boost = max(self.boost, 0.0)

        self.current_speed += self.speed_pid.update(speed_error)

        # # for discrete action qlearning
        # self.steering_angles = [-0.3, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 0.3]
        # # round to nearest steering angle
        # steering = min(self.steering_angles, key=lambda x: abs(x - steering))

        return (self.current_speed, steering)






