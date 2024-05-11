import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32

from sensor_msgs.msg import LaserScan
from my_messages.msg import PIDInput, KeyValue

class FieldView(Node):
    def __init__(self):
        super().__init__('field_view')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.publisher_ = self.create_publisher(PIDInput, '/error', 10)
        self.distance_publisher = self.create_publisher(Float32, '/visual/rightdist', 10)
        self.acceleration_predictor = self.create_publisher(Float32, '/decision', 10)

        #self.right_wall = self.create_publisher(Float32, '/right', 10)
        #self.left_wall = self.create_publisher(Float32, '/left', 10)

        # lidar beam index from the rightmost side
        self.wall_projection_r = 350
        # lidar beam index from the leftmost side
        self.wall_projection_l = 730

        self.wall_projection_area = range(350,731)

        # since the reference has been halved, the target needs to follow suit
        self.target_distance = 0.7

        # initialize the steering error
        self.error = 0.0

    def callback(self, msg):

        beam_dict = {}
        

        # reducing the sensativity of the reference by halving the distance of the beam
        for beam in self.wall_projection_area:
            wall_distance = (0.5)*((msg.ranges)[beam])
            
                # self.get_logger().info(f"Publishing 540 Distance: \"{wall_distance}\"")

            # elif beam == self.wall_projection_r:
            #     # error of about 0.04
            #     float32_value = Float32()
            #     float32_value.data = wall_distance
            #     self.right_wall.publish(float32_value)

            # elif beam == self.wall_projection_l:
            #     float32_value = Float32()
            #     float32_value.data = wall_distance
            #     self.left_wall.publish(float32_value)


            if wall_distance > 1.5:
                wall_distance = 1.5

            # Used for Painter
            if beam == 540:
                float32_msg = Float32()
                float32_msg.data = wall_distance
                self.distance_publisher.publish(float32_msg)

        # calculate the error - difference between the value target and the real one
            beam_dict[beam] = wall_distance
        # An empty msg is created of the type pid_input
        pid_msg = PIDInput()

        # this is the error that you want to send to the PID for steering correction.
        for beamer, steering_errors in beam_dict.items():
            pid_msg.pid_error.append(KeyValue(key=beamer, value=steering_errors))

        self.publisher_.publish(pid_msg)

    
def main(args=None):
    rclpy.init(args=args)

    plexer360 = FieldView()

    rclpy.spin(plexer360)

    plexer360.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()