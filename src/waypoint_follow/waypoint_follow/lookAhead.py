import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from nav_msgs.msg import Path
# from .WaypointHandler import WaypointHandler
from .WaypointNavigator import WaypointNavigator

class lookAhead(Node):
    def __init__(self):
        super().__init__('lookAhead')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Look a head Node has been started.")

        self.waypoint_topic = "/centerline_waypoints"
        self.waypoint_subscriber = self.create_subscription(
            Path, self.waypoint_topic, self.waypoint_callback, 10
        )
        self.get_logger().info("waiting_for_waypoint")

        self.navigator = WaypointNavigator()
        self.got_waypoints = False


    def waypoint_callback(self, msg):
        if(not self.got_waypoints):
            self.get_logger().info("Received waypoints for the first time!")
            self.got_waypoints = True

        # continue updating in case path changes for obstacles
        self.navigator.waypoint_handler.update_waypoints(msg)


    def timer_callback(self):

        #next_waypoint, angle_to_waypoint_world, waypoint_angle, distance_to_waypoint = self.waypoint_handler.get_next_waypoint(car_pos, car_heading_world, lookahead, use_direction)
        next_waypoint = self.navigator.waypoint_handler.get_next_waypoint(car_pos, car_yaw)

        if next_waypoint is None:
            return (0.0, 0.0) 

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ""
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = next_waypoint.pose.position.x
        marker.pose.position.y = next_waypoint.pose.position.y
        marker.pose.position.z = next_waypoint.pose.position.z
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = lookAhead()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()