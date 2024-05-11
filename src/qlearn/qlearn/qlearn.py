import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Header
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

from std_srvs.srv import Trigger, SetBool
from qlearn_interfaces.srv import SaveQTable, LoadQTable, SetFloat, SetInt
from qlearn_interfaces.msg import FloatStamped, IntStamped 

import torch
import random
import pickle
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from matplotlib import pyplot as plt

from .QTable import QTable
from .QNetwork import QNetwork
from .ReplayBuffer import ReplayBuffer


class QLearn(Node):
    def __init__(self):
        super().__init__("qlearn")
        
        # Subscribers
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.scan_data = None

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.odom_data = None

        self.create_subscription(Path, "/centerline_waypoints", self.waypoint_callback, 10)
        self.waypoint_data = None

        self.create_subscription(Bool, "/ego_collision", self.collision_callback, 10)
        self.is_collision = False

        self.create_subscription(AckermannDriveStamped, "/drive", self.drive_callback, 10)
        self.drive_data = None
        self.use_imitation = False

        self.create_subscription(Float32, "/ego_lap_time", self.lap_time_callback, 10)
        self.lap_time = 0.0
        
        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.reset_pub = self.create_publisher(PoseWithCovarianceStamped,"/initialpose", 10)
        self.reward_pub = self.create_publisher(FloatStamped, "/qlearn_reward", 10)
        self.step_pub = self.create_publisher(IntStamped, "/qlearn_step", 10)
        self.episodes_since_collision_pub = self.create_publisher(IntStamped, "/qlearn_episodes_since_collision", 10)
        self.loss_st_pub = self.create_publisher(FloatStamped, "/qlearn_loss_steering", 10)
        self.loss_sp_pub = self.create_publisher(FloatStamped, "/qlearn_loss_speed", 10)
        self.speed_pub = self.create_publisher(FloatStamped, "/qlearn_speed", 10)
        self.steering_pub = self.create_publisher(FloatStamped, "/qlearn_steering", 10)


        # Services
        self.create_service(SaveQTable, '/save_qtable', self.save_qtable_callback)
        self.create_service(LoadQTable, '/load_qtable', self.load_qtable_callback)
        self.create_service(SetFloat, '/set_learning_rate', self.set_learning_rate_callback)
        self.create_service(SetInt, '/set_stage', self.set_stage_callback)

        self.create_service(Trigger, '/toggle_imitation', self.toggle_imitation_callback)
        self.create_service(Trigger, '/toggle_random', self.toggle_random_callback)
        self.create_service(Trigger, '/toggle_limitless', self.toggle_limitless_callback)
        self.create_service(Trigger, '/toggle_log_qvalues', self.toggle_log_qvalues_callback)
        self.create_service(Trigger, '/toggle_update', self.toggle_update_callback)
        self.create_service(Trigger, '/toggle_difficults', self.toggle_difficults_callback)

        # create timer
        self.create_timer(1/160, self.step_call)  # default: 20Hz
        self.start_speed = 6.0
        self.min_speed = 2.0
        self.top_speed = 10.0

        self.use_table = False
        if self.use_table:
            self.q_table = QTable(learning_rate = 0.1, random_rate = 0.9, discount_rate = 0.9, lamda = 10)
        else:
            self.q_table = QNetwork(learning_rate = 0.0016, random_rate = 0.8, discount_rate = 0.99, top_speed=self.top_speed)

        self.use_limitless = False
        self.use_randomness = True
        self.use_random_spawn = True
        self.use_difficulty_accumulation = False
        self.log_qvalues = False
        self.update_enabled = True

        self.episode_count = 0
        self.step_count = 1
        self.max_step_count = 10
        self.stage = -1

        self.episode_wp = None
        self.episode_rewards = []
        self.episode_speed_acc = 0.0
        self.episode_steerings = []
        self.difficults = []
        self.accumulate_difficults = True  # accumulate 10 episodes, then practice on most difficult, this variable determines whether we are currently collecting or practicing
        self.rewards = []
        self.lap_times = []
        self.episodes_since_last_collision = 0
        self.last_car = None
        self.start_time = self.get_clock().now()
        self.last_state = None
        self.last_action = None
        self.last_ack_action = None

        
    def step_call(self):
        if self.waypoint_data == None:
            self.get_logger().info("Waiting for waypoints...")
            return
        if self.scan_data == None or self.odom_data == None:
            self.get_logger().info("############### QLearning is running faster than the simulation... ###############")
            return

        speed = self.last_ack_action[1] if self.last_ack_action is not None else self.start_speed
        yaw = self.get_yaw(self.odom_data.pose)

        state = self.q_table.get_state(self.scan_data, speed, yaw)
        q_values_st, q_values_sp = self.q_table.get_q_values(state)

        if self.log_qvalues:
            print(q_values)

        action = 0
        action_ack = [0.0, 0.0]
        if self.use_imitation:
            if not self.drive_data:
                self.get_logger().info("Waiting for imitation data...")
                return
            action = self.q_table.ackermann_to_action_index(self.drive_data)
            action_ack = self.q_table.get_action_ack(action, speed, self.top_speed, self.min_speed)
        else:
            action = self.q_table.get_action_index(q_values_st, q_values_sp, self.use_randomness)
            action_ack = self.q_table.get_action_ack(action, speed, self.top_speed, self.min_speed)

        wp = self.get_closest_waypoint()
        car = ((self.odom_data.pose.pose.position.x, self.odom_data.pose.pose.position.y), yaw, speed)
        if self.last_car is None:
            self.last_car = car

        reward = self.q_table.get_reward(speed, car, self.last_car, wp, self.is_collision)
        self.episode_rewards.append(reward)
        self.episode_speed_acc += abs(action_ack[1])
        self.episode_steerings.append(action_ack[0])

        if self.last_state is not None:
            self.q_table.replay_buffer.add(self.last_state, self.last_action[0], self.last_action[1], reward, state, int(self.is_collision))  # add to experience buffer

        if self.step_count % 20 == 0:
            # self.get_logger().info("Step {}, Action: ( s: {}, v: {} ), Reward: {}".format(self.step_count, action_ack[0], action_ack[1], round(reward, 2)))
            if self.use_imitation:
                self.get_logger().info("Learning from imitation data")


        # update q table using for s using s, a, r, s'
        if self.use_table and self.update_enabled:
            self.q_table.update(state)
        # else, update is done at the end of the episode

        if self.is_collision:
            self.end_episode()
            return

        self.step_count += 1
        self.last_car = car
        self.last_state = state
        self.last_action = action
        self.last_ack_action = action_ack

        if self.step_count >= self.max_step_count and not self.use_limitless:
            self.end_episode()
            return

        if not self.use_imitation:
            self.publish_ackermann_msg(action_ack)

        self.reset_step()


    def reset_step(self):
        self.odom_data = None
        self.scan_data = None

    def reset_episode(self):

        if not self.use_difficulty_accumulation or self.accumulate_difficults or len(self.difficults) == 0:
            if self.stage < 0:
                x, y, theta = self.get_random_waypoint()
                if random.random() < 0.5:
                    # flip direction
                    theta += np.pi
            else:
                if self.stage == 0:
                    wp = self.waypoint_data.poses[1]
                elif self.stage == 1:
                    wp = self.waypoint_data.poses[-60]
                elif self.stage == 2:
                    wp = self.waypoint_data.poses[580]

                x, y, theta = wp.pose.position.x, wp.pose.position.y, self.get_yaw(wp)

            if self.use_random_spawn:
                theta += random.random() * 0.4 - 0.2  # add noise to theta
                theta = theta % (2 * np.pi)

                x += random.random() * 1.2 - 0.6  # add noise to x
                y += random.random() * 1.2 - 0.6  # add noise to y
        else:
            x, y, theta = self.difficults[0][0]  # practice on most difficult

        self.episode_wp = (x, y, theta)

        self.set_car(x, y, theta)

        if self.is_collision:
            self.episodes_since_last_collision = 0

        self.is_collision = False
        self.last_car = None
        self.last_state = None
        self.last_action = None
        self.last_ack_action = None

        self.step_count = 1
        self.episode_wp = None
        self.episode_rewards = []
        self.episode_speed_acc = 0.0
        self.episode_steerings = []

    def end_episode(self):

        self.episode_count += 1

        # Statistics
        avg_reward = round(sum(self.episode_rewards) / self.step_count, 4)
        self.rewards.append(avg_reward)
        self.lap_times.append(self.lap_time)

        if not self.is_collision:
            self.episodes_since_last_collision += 1

        # publish
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        if self.episode_count % 4 == 0:
            self.reward_pub.publish(FloatStamped(header=header, data=avg_reward))
            self.step_pub.publish(IntStamped(header=header, data=self.step_count))
            avg_speed = round(self.episode_speed_acc / self.step_count, 4)
            avg_steering = round(sum(self.episode_steerings) / self.step_count, 4)
            self.speed_pub.publish(FloatStamped(header=header, data=avg_speed))
            self.steering_pub.publish(FloatStamped(header=header, data=avg_steering))
            if not self.is_collision:
                self.episodes_since_collision_pub.publish(IntStamped(header=header, data=self.episodes_since_last_collision))
            else:
                self.episodes_since_collision_pub.publish(IntStamped(header=header, data=0))

        # finish updating q table
        loss = None
        if self.use_table and self.update_enabled:
            if len(self.q_table.rewards) > 0:
                sp = self.q_table.states.pop()
                while len(self.q_table.rewards) >= 0:
                    loss = self.q_table.update(sp)
                    if len(self.q_table.states) == 0:
                        break
                    # remove first elements
                    self.q_table.states = self.q_table.states[1:]
                    self.q_table.actions = self.q_table.actions[1:]
                    self.q_table.rewards = self.q_table.rewards[1:]
        else:
            if self.update_enabled:
                loss_st, loss_sp = self.q_table.update()
                if loss_st != None:
                    loss = loss_st + loss_sp
                    loss = round(loss, 4)
                    loss_st = round(loss_st, 4)
                    loss_sp = round(loss_sp, 4)

                    if self.episode_count % 3 == 0:
                        self.loss_st_pub.publish(FloatStamped(header=header, data=loss_st))
                        self.loss_sp_pub.publish(FloatStamped(header=header, data=loss_sp))

        # difficults
        if self.accumulate_difficults and self.episode_wp != None:
            if self.is_collision and not self.difficult_exists(self.episode_wp):
                self.difficults.append([self.episode_wp, 1])

            if self.episode_count % 100 == 0:
                self.accumulate_difficults = False
        else:
            if self.episode_count % 25 == 0:
                self.difficults = []
                self.accumulate_difficults = True

        # tweak hyperparams
        if self.max_step_count < 200 and self.episode_count % 18 == 0:
            self.max_step_count += 1
        if self.q_table.random_rate > 0.01 and self.use_randomness:
            self.q_table.random_rate /= 1.002

        lr = self.q_table.optimizer.param_groups[0]['lr']
        self.get_logger().info("Episode {}: Avg L: {}, s: {}/{}, lr: {}, rr: {}, bi: {}".format(self.episode_count, loss, self.step_count, self.max_step_count, lr, round(self.q_table.random_rate,3), self.q_table.replay_buffer.buffer_index))

        self.reset_episode()


    def get_random_waypoint(self):
        # returns x, y, theta
        if not self.waypoint_data:
            return 0.0, 0.0, 0.0

        waypoint = self.waypoint_data.poses[random.randint(0, len(self.waypoint_data.poses) - 1)]
        # get yaw from quat
        yaw = self.get_yaw(waypoint)

        return waypoint.pose.position.x, waypoint.pose.position.y, yaw
    
    def get_closest_waypoint(self):
        if not self.waypoint_data or not self.odom_data:
            return ((0.0, 0.0), 0)
        closest_dist = 1000000
        closest_waypoint = self.waypoint_data.poses[0]
        for i in range(len(self.waypoint_data.poses)):
            waypoint = self.waypoint_data.poses[i]
            dist = (waypoint.pose.position.x - self.odom_data.pose.pose.position.x)**2 + (waypoint.pose.position.y - self.odom_data.pose.pose.position.y)**2
            if dist < closest_dist:
                closest_dist = dist
                closest_waypoint = waypoint
        # get yaw from quat
        yaw = self.get_yaw(closest_waypoint)
        return ((closest_waypoint.pose.position.x, closest_waypoint.pose.position.y), yaw)

    def get_yaw(self, data):
        return euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]

    def difficult_exists(self, wp):
        # check if waypoint is in difficults (within 5m)
        for i in range(len(self.difficults)):
            if (wp[0] - self.difficults[i][0][0])**2 + (wp[1] - self.difficults[i][0][1])**2 < 25 and (wp[2] - self.difficults[i][0][2]) < np.pi:
                self.difficults[i][1] += 1
                return True
        return False

    def sort_difficults(self):
        # where difficults is [(wp, count), ...] and first is most difficult
        self.difficults.sort(key=lambda x: x[1], reverse=True)

    def set_car(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # euler to quat
        quat = quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        self.reset_pub.publish(msg)

    def publish_ackermann_msg(self, ack_action):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = ack_action[0]
        msg.drive.speed = ack_action[1]
        self.drive_pub.publish(msg)

    def scan_callback(self, msg):
        self.scan_data = msg

    def odom_callback(self, msg):
        self.odom_data = msg

    def waypoint_callback(self, msg):
        self.waypoint_data = msg

    def collision_callback(self, msg):
        if msg.data:  # if collision, keep it true until next step
            self.is_collision = msg.data
            self.publish_ackermann_msg((0.0, 0.0))

    def drive_callback(self, msg):
        self.drive_data = msg

    def lap_time_callback(self, msg):
        self.lap_time = msg.data


    def toggle_limitless_callback(self, _, response):
        self.use_limitless = not self.use_limitless
        response.success = True
        response.message = "Limitless: {}".format(self.use_limitless)
        return response

    def toggle_random_callback(self, _, response):
        self.use_randomness = not self.use_randomness
        response.success = True
        response.message = "Randomness: {}".format(self.use_randomness)
        return response

    def toggle_imitation_callback(self, _, response):
        self.use_imitation = not self.use_imitation
        response.success = True
        response.message = "Imitation: {}".format(self.use_imitation)
        self.drive_data = None
        return response
    
    def toggle_log_qvalues_callback(self, _, response):
        self.log_qvalues = not self.log_qvalues
        response.success = True
        response.message = "Log QValues: {}".format(self.log_qvalues)
        return response

    def toggle_update_callback(self, _, response):
        self.update_enabled = not self.update_enabled
        response.success = True
        response.message = "Update: {}".format(self.update_enabled)
        return response

    def toggle_difficults_callback(self, _, response):
        self.use_difficulty_accumulation = not self.use_difficulty_accumulation
        response.success = True
        response.message = "Difficults: {}".format(self.use_difficulty_accumulation)
        return response
    

    def set_learning_rate_callback(self, request, response):
        try:
            self.q_table.set_learning_rate(request.data)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to set learning rate: {e}")
            response.success = False
        return response

    def set_stage_callback(self, request, response):
        self.stage = request.data
        response.success = True
        return response

    def save_qtable_callback(self, request, response):
        try:
            if self.use_table:
                with open(request.filename, 'wb') as file:
                    pickle.dump(self.q_table.q_table, file)
            else:
                torch.save(self.q_table.model.state_dict(), request.filename)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to save QTable: {e}")
            response.success = False
        return response

    def load_qtable_callback(self, request, response):
        try:
            if self.use_table:
                with open(request.filename, 'rb') as file:
                    new_table = pickle.load(file)
                    self.load_into_array(new_table, self.q_table.q_table)
            else:
                state_dict = torch.load(request.filename)
                self.q_table.model.load_state_dict(state_dict)
                self.q_table.target_model.load_state_dict(state_dict)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to load QTable: {e}")
            response.success = False
        return response

    def load_into_array(self, source_array, target_array):
        # Calculate the slice size for each dimension
        slice_sizes = tuple(slice(0, min(src_dim, tgt_dim)) for src_dim, tgt_dim in zip(source_array.shape, target_array.shape))
        
        # Load data from source_array into target_array
        target_array[slice_sizes] = source_array[slice_sizes]


def main(args=None):
    rclpy.init(args=args)
    
    qlearn = QLearn()
    
    rclpy.spin(qlearn)
    
    qlearn.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
