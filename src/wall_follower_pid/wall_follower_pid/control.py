import rclpy
import math
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from pid_msgs.msg import PIDInput

class PIDController(Node):

    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(
            PIDInput, 
            '/error', 
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 1)

        self.prev_steering_error = 0.0
        
        # velocity value
        self.vel_input = 4.0

        # PID params
        self.s_kp = 0.8
        self.s_kd = 0.2
        self.s_ki = 0.0

    def callback(self, msg):
        
        self.get_logger().info(f"Subscribing Steering Error: \"{msg.pid_error}\"")

        # grab the error from the WallDistanceFinder node
        steering_error = msg.pid_error
        
        # Calculate the error derivative (change in error)
        steering_error_diff = steering_error - self.prev_steering_error
        
        # Update the previous error for the next iteration
        self.prev_steering_error = steering_error
        
        # calculate desired steering angle from steering error (P and D)
        angle = self.s_kp * steering_error + self.s_kd * steering_error_diff
        
        # saturate the steering angle
        angle = max(min(angle, 0.85), -0.85)

        # initialize the Ackermann message
        stamped_command = AckermannDriveStamped()

        # modify the steering angle element of the /drive message
        stamped_command.drive.steering_angle = angle

        # modify the velocity of the /drive message 
        stamped_command.drive.speed = self.vel_input

        # publish the message (includes the modified elements)
        self.publisher_.publish(stamped_command)

        self.get_logger().info(f"Publishing Steering Angle: \"{stamped_command.drive.steering_angle}\"")
        
        self.get_logger().info(f"Publishing Speed: \"{stamped_command.drive.speed}\"")

def main(args=None):
    rclpy.init(args=args)

    pid_controller = PIDController()

    rclpy.spin(pid_controller)

    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
