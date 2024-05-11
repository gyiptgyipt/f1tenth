import rclpy
import math
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from my_messages.msg import PIDInput
from std_msgs.msg import Float32
from std_msgs.msg import Bool


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(PIDInput, '/error', self.callback, 10)
        self.acc_subscription = self.create_subscription(Float32, '/decision', self.accelerator_callback, 10)
        #self.right_wall = self.create_subscription(Float32, '/right', self.right_callback, 10)
        #self.left_wall = self.create_subscription(Float32, '/left', self.left_callback, 10)

        self.car_vel = self.create_subscription(Odometry, '/odom', self.car_vel_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.action = self.create_subscription(Float32, '/action', self.action_callback, 10)

        self.prev_steering_error = 7.0
        
        # constant velocity value
        self.vel_input = 0.0

        self.a = 2.0
        self.d = 0.1
        self.max_vel = 15.0
        self.min_dist = 4.5

        self.action = 1

        #self.right_wall = 0.0
        #self.left_wall = 0.0

        #self.speed = 0.0

        self.acc = 0.0

        self.car_vel = 0.0

        # PID params
        self.s_kp = 0.2 #TODO
        self.s_kd = 0.6 #TODO
        self.s_ki = 0.1 #TODO
    
    def callback(self, msg):
        

        right_side_sum = 0
        left_side_sum = 0
        middle_beam = 0

        # grab the error from the WallDistanceFinder node
        for kv_pair in msg.pid_error:
            beam = kv_pair.key
            wall_distance = kv_pair.value

            if 350 <= beam < 540:
                right_side_sum += wall_distance
            elif 540 < beam <= 730:
                left_side_sum += wall_distance
            else:
                middle_beam = middle_beam + wall_distance


        
        # Calculate the error derivative (change in error)
        steering_error = left_side_sum - right_side_sum

        # calculate desired steering angle from steering error (P and D)
        angle = (self.s_kp * steering_error) + (self.s_kd * (steering_error - self.prev_steering_error))
        angle = angle/100
        
    
        stamped_command = AckermannDriveStamped()

                
        # car_vel = self.car_vel
        # if car_vel == 0:
        #     time = 0.0
        # else:
        #     time = self.acc/11.5

        # def scale_speed(time):
        #     time_range = [0.1, 2.0]
        #     speed_range = [4.0, 13.0]

        #     if time >= 0.5:
        #         return 13.0
            
        #     speed = (time - time_range[0]) / (0.5 - time_range[0]) * (speed_range[1] - speed_range[0]) + speed_range[0]
        #     return max(min(speed, speed_range[1]), speed_range[0])

        velocity = self.car_vel

        if self.action == 0.0:
            velocity += 2.0
            stamped_command.drive.speed = velocity
        elif self.action == 1.0:
            stamped_command.drive.speed = velocity
        else:
            velocity -= 0.5
            if velocity <= 0.0:
                velocity = 0.0
            stamped_command.drive.speed = velocity
        
        angle = max(min(angle, 0.85), -0.85)
        self.prev_steering_error = steering_error
        stamped_command.drive.steering_angle = angle

        

        # publish the message (includes the modified elements)
        self.publisher_.publish(stamped_command)

    
    def accelerator_callback(self, acc):
        self.acc = acc.data
    
    # def right_callback(self, right_wall):
    #     self.right_wall = right_wall.data

    # def left_callback(self, left_wall):
    #     self.left_wall = left_wall

    def car_vel_callback(self, car_vel):
        car_vel1 = car_vel.twist
        car_vel2 = car_vel1.twist
        car_vel3 = car_vel2.linear
        self.car_vel = car_vel3.x

    def action_callback(self, action):
        self.action = action.data
        


def main(args=None):
    rclpy.init(args=args)

    pid_controller = PIDController()

    rclpy.spin(pid_controller)

    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
