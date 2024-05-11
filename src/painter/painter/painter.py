import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import tf_transformations  # should be installed otherwise apt install ros-iron-tf-transformations

import math


class PainterNode(Node):
    def __init__(self):
        super().__init__("painter_node")
        self._line_publisher = self.create_publisher(Marker, "/line_test", 10)

        # subscribe to /odom
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self._i = 0

    def odom_callback(self, msg):
        self._i += 1
        self.get_logger().info("Publishing line %d" % self._i)

        pose = msg.pose.pose
        car_pos = (pose.position.x, pose.position.y)
        car_speed = msg.twist.twist.linear.x
        car_steering = msg.twist.twist.angular.z

        # car direction
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        car_direction = euler[2]  # radians

        scale = car_speed * 0.20
        draw_point = (
            car_pos[0]
            + math.cos(car_direction) * scale,
            car_pos[1]
            + math.sin(car_direction) * scale,
        )

        self.draw_line(
            car_pos,
            draw_point,
            id=0,
            color=(1.0, 1.0, 0.0),
        )

    def draw_line(self, c1, c2, id=0, color=(0.0, 1.0, 0.0)):
        # p1, p2 to Points
        p1 = Point()
        p1.x = c1[0]
        p1.y = c1[1]
        p1.z = 0.1
        p2 = Point()
        p2.x = c2[0]
        p2.y = c2[1]
        p2.z = 0.1

        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = id
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.points = [p1, p2]
        marker.pose.orientation.w = 1.0
        scale = 0.1  # adjust if needed
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        self._line_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    painter_node = PainterNode()

    rclpy.spin(painter_node)

    painter_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
