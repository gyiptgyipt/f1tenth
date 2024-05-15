import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from ackermann_msgs.msg import AckermannDriveStamped

from .WaypointNavigator import WaypointNavigator


class FollowWaypoint(Node):
    """
    Minimal publisher class
    """

    def __init__(self):
        super().__init__("waypoint_follow")
        self.get_logger().info("Waypoint Follow Node Started")

        # publish to a car's drive
        self.drive_topic = "/drive"
        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1
        )
        self.get_logger().info("Publishing to %s" % self.drive_topic)

        # subscribe to a car's odom
        self.odom_topic = "/odom"
        self.subscriber = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # subscribe to scan
        self.scan_topic = "/scan"
        self.scan_subscriber = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )

        # subscribe to waypoint publisher
        self.waypoint_topic = "/centerline_waypoints"
        self.waypoint_subscriber = self.create_subscription(
            Path, self.waypoint_topic, self.waypoint_callback, 10
        )
        self.get_logger().info("Waiting for waypoints...")

        # how often we will send commands to the car
        timer_period = 1/80  # 80Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # waypoint navigator
        self.navigator = WaypointNavigator()

        self.car_odom = Odometry()
        self.got_odom = False

        self.car_scan = LaserScan()
        self.got_scan = False

        self.got_waypoints = False

    def timer_callback(self):
        """
        Drive timer callback
        """
        if not self.got_odom or not self.got_scan or not self.got_waypoints:
            return

        timestamp = self.get_clock().now()

        speed, steering_angle = self.navigator.get_drive(self.car_odom, self.car_scan)

        msg = AckermannDriveStamped()
        msg.header.stamp = timestamp.to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle

        self.publisher.publish(msg)

        self.got_odom = False  # we want odom to be fresh every time we run

    def odom_callback(self, msg):
        self.car_odom = msg
        self.got_odom = True

    def scan_callback(self, msg):
        self.car_scan = msg
        self.got_scan = True

    def waypoint_callback(self, msg):
        if(not self.got_waypoints):
            self.get_logger().info("Received waypoints for the first time!")
            self.got_waypoints = True

        # continue updating in case path changes for obstacles
        self.navigator.waypoint_handler.update_waypoints(msg)


def main(args=None):
    """
    Main
    """
    rclpy.init(args=args)

    follow_waypoint = FollowWaypoint()
    rclpy.spin(follow_waypoint)

    follow_waypoint.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
