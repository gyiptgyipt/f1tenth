import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32


class Multiplexer(Node):
    def __init__(self):
        super().__init__('multiplexer')
        self.acc_subscription = self.create_subscription(Float32, '/decision', self.decision, 10)
        self.right_wall = self.create_subscription(Float32, '/right', self.right_callback, 10)
        self.left_wall = self.create_subscription(Float32, '/left', self.left_callback, 10)


        self.permission = self.create_publisher(Bool, '/permission', 1)

        self.right_wall = 0.0
        self.left_wall = 0.0


    def decision(self, msg):
        
        self.acc = msg.data

        if self.acc >= 4.5:
            msg = Bool()
            msg.data = True
            self.permission.publish(msg)
        else:
            msg = Bool()
            msg.data = False
            self.permission.publish(msg)

    def right_callback(self, right_wall):
        self.right_wall = right_wall.data

    def left_callback(self, left_wall):
        self.left_wall = left_wall.data


def main(args=None):
    rclpy.init(args=args)

    multiplexer = Multiplexer()

    rclpy.spin(multiplexer)

    multiplexer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()