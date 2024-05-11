
import math

from tf_transformations import euler_from_quaternion, quaternion_from_euler

class WaypointHandler:
    def __init__(self):
        self.waypoints = None
        self.car_length = 0.3302

    def get_next_waypoint(self, car_pos, car_yaw, dist = 0.6604, use_dir = False):
        """
        Returns next waypoint in front of the car
        """
        if(self.waypoints is None):
            return None, 0.0, 0.0

        # distance of point inf front of car based
        closest_waypoint = None
        closest_distance = float('inf')

        if not use_dir:
            point_in_front = (car_pos[0] + math.cos(car_yaw) * dist, car_pos[1] + math.sin(car_yaw) * dist)
            for waypoint in self.waypoints.poses:
                distance = math.sqrt((waypoint.pose.position.x - point_in_front[0])**2 + (waypoint.pose.position.y - point_in_front[1])**2)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_waypoint = waypoint
        else:
            # pick closest waypoint in direction of car
            search_angle = 1.5  # 90 degrees
            min_dist = self.car_length * 2.0
            for waypoint in self.waypoints.poses:
                dx = waypoint.pose.position.x - car_pos[0]
                dy = waypoint.pose.position.y - car_pos[1]
                angle = math.atan2(dy, dx)
                relative_angle = self.get_relative_angle(car_yaw, angle)
                distance = math.sqrt(dx**2 + dy**2)

                if abs(relative_angle) < search_angle and distance < closest_distance and distance > min_dist:
                    closest_distance = distance
                    closest_waypoint = waypoint

        if closest_waypoint is None:
            return None, 0.0, 0.0, 0.0

        angle, distance = self.get_target(car_pos, closest_waypoint)
        waypoint_angle = euler_from_quaternion((closest_waypoint.pose.orientation.x, closest_waypoint.pose.orientation.y, closest_waypoint.pose.orientation.z, closest_waypoint.pose.orientation.w))[2]

        return closest_waypoint, angle, waypoint_angle, distance

    def angle_check(self,angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_relative_angle(self, car_yaw, angle):
        relative_angle = angle - car_yaw
        relative_angle = self.angle_check(relative_angle)
        return relative_angle

    def get_target(self, car_pos, next_waypoint):
        """
        Returns angle from car position to waypoint position and distance between
        """
        dx = next_waypoint.pose.position.x - car_pos[0]
        dy = next_waypoint.pose.position.y - car_pos[1]
        return math.atan2(dy, dx), math.sqrt(dx**2 + dy**2)

    def update_waypoints(self, waypoints):
        self.waypoints = waypoints

