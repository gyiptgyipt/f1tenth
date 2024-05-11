import rclpy
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from pid_msgs.msg import PIDInput

class WallDistanceFinder(Node):

    def __init__(self):
        super().__init__('wall_distance_finder')
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(PIDInput, '/error', 10)
        
        # lidar beam index from the rightmost side
        self.wall_projection_r = 350

        # since the reference has been halved, the target needs to follow suit
        self.target_distance = 0.7
        
        # initialize the steering error
        self.error = 0.0

    def callback(self, msg): 

        # reducing the sensitivity of the reference by halving the distance of the beam
        wall_distance_r = (0.5)*((msg.ranges)[self.wall_projection_r])

        # calculate the error - difference between the value target and the real one
        steering_error = self.target_distance - wall_distance_r

        # An empty msg is created of the type pid_input
        pid_msg = PIDInput()
        
        # this is the error that you want to send to the PID for steering correction.
        pid_msg.pid_error = steering_error
        
        self.publisher_.publish(pid_msg)
        
        self.get_logger().info(f"Publishing Steering Error: \"{pid_msg.pid_error}\"")
    
def main(args=None):
    rclpy.init(args=args)

    wall_distance_finder = WallDistanceFinder()

    rclpy.spin(wall_distance_finder)

    wall_distance_finder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
