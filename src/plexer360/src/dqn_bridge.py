import rclpy
from rclpy.node import Node
from my_messages.msg import PIDInput
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float32



from src.dqn import DQN

class DQN_Bridge(Node):
    def __init__(self):
        super().__init__('dqn_bridge')

        self.subscription = self.create_subscription(PIDInput, '/error', self.lidar_callback, 10)
        self.car_vel = self.create_subscription(Odometry, '/odom', self.car_vel_callback, 10)
        self.decision = self.create_publisher(Float32, '/action', 10)

        self.car_vel = 0.0

        self.env = None

        self.num_actions = 3
        self.new_size = 0

        self.reward = 0.0
        self.dec = 0
        self.first_state = None
        self.done = False
        
        self.dqn = None

        self.counter = 0

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.action()

    def action(self):
        
        if self.env == None:
            print("Environment not initialized. Skipping action.")
            return
        
        right, left, mid = self.env

        state_size = np.array([ int(right), int(left), int(mid), int(self.car_vel)])

        if self.counter == 0:
            print("Not Training Model")
            self.counter += 1
            self.first_state = state_size
        else:
            print("Check to Train")
            if self.first_state is not None:
                print("Training model")
                self.dqn.train(self.first_state, self.dec, self.reward, state_size, self.done)
        
        if self.dqn is None:
            # Initialize DQN if not initialized yet
            
            self.dqn = DQN(state_size.size, self.num_actions)
            

        
        action = self.dqn.select_action(state_size)
        self.dec = action
        this_action = float(action)
        float32_msg = Float32()
        float32_msg.data = this_action
        self.decision.publish(float32_msg)

        velocity = self.car_vel
        
        if velocity <= 0.0:
            self.reward -= 20.0
        elif velocity <= 4.0:
            self.reward -= 8.0
        elif velocity >= 18.0:
            self.reward -= 3.0
        else:
            self.reward += 10.0
        
        print(self.reward)

        self.done = False



    def lidar_callback(self, msg):

        right_side_sum = 0
        left_side_sum = 0
        middle_beam = 0

        lidar_data = msg.pid_error
        for kv_pair in lidar_data:
            beam = kv_pair.key
            wall_distance = kv_pair.value

            if 350 <= beam < 540:
                right_side_sum += wall_distance
            elif 540 < beam <= 730:
                left_side_sum += wall_distance
            else:
                middle_beam = middle_beam + wall_distance

        self.env = [right_side_sum, left_side_sum, middle_beam]



    def car_vel_callback(self, car_vel):
        car_vel1 = car_vel.twist
        car_vel2 = car_vel1.twist
        car_vel3 = car_vel2.linear
        self.car_vel = car_vel3.x

def main(args=None):
    
    rclpy.init(args=args)

    dqn_node = DQN_Bridge()
    
    rclpy.spin(dqn_node)
    
    dqn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()