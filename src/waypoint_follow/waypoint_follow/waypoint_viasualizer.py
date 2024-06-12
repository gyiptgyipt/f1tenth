import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from .WaypointNavigator import WaypointNavigator  # Import the WaypointNavigator from the library

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')
        self.waypoint_navigator = WaypointNavigator()
        self.marker_publisher = self.create_publisher(Marker, 'waypoint_marker', 10)
        self.marker_id = 0

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.current_odom = None
        self.current_scan = None

    def odom_callback(self, msg):
        self.current_odom = msg
        self.process_data()

    def scan_callback(self, msg):
        self.current_scan = msg
        self.process_data()

    def process_data(self):
        if self.current_odom is not None and self.current_scan is not None:
            (speed, steering), next_waypoint_pos = self.waypoint_navigator.get_drive(self.current_odom, self.current_scan)
            if next_waypoint_pos is not None:
                self.publish_waypoint_marker(next_waypoint_pos)

    def publish_waypoint_marker(self, waypoint_position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = waypoint_position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    waypoint_visualizer = WaypointVisualizer()
    rclpy.spin(waypoint_visualizer)
    waypoint_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
