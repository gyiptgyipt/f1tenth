
import math
import random
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .ReplayBuffer import ReplayBuffer

class QTable:
    def __init__(self, learning_rate=0.1, discount_rate=0.9, random_rate=0.2, lamda = 20):
        
        self.lamda = lamda  # look ahead depth, lambda is reserved

        self.action_dim = 7
        self.q_table = np.random.rand(20, 20, 20, 20, 20, 1, self.action_dim)  # (...distances, v, action)

        self.states = []
        self.actions = []
        self.rewards = []
        self.replay_buffer = ReplayBuffer(0)  # only used for DQN
        self.model = nn.Sequential()  # only used for DQN

        self.learning_rate = learning_rate
        self.discount_rate = discount_rate
        self.random_rate = random_rate

        self.steering_angles = [-0.3, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 0.3]
        self.speeds = [float(x) for x in range(4, 5, 2)]  # 3
        self.action_map = {}
        for i in range(self.action_dim):
            self.action_map[i] = (self.steering_angles[i % len(self.steering_angles)], self.speeds[i // len(self.steering_angles)])

        print("Action map:")
        print(self.action_map)


    def get_state(self, scan_data):
        # 20 rays ~ 5 degrees
        return self.clamp_state([ scan_data.ranges[540+180], scan_data.ranges[540 + 90], scan_data.ranges[540], scan_data.ranges[540-90], scan_data.ranges[540-180] ]) 

    def get_q_values(self, state):
        loc = tuple(state) + (0,)
        return self.q_table[loc]

    def get_action(self, q_values, use_randomness=True):
        if use_randomness and random.random() < self.random_rate:
            return random.randint(0, self.action_dim - 1)  # off policy
        else:
            return np.argmax(q_values)  # on policy

    def update_states_actions_rewards(self, state, action, reward):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        if len(self.states) > self.lamda:
            self.states = self.states[1:]
            self.actions = self.actions[1:]
            self.rewards = self.rewards[1:]

    def update(self, state_prime=None):
        if len(self.states) == 0:
            return None
        if state_prime is None:
            print("ERROR: state_prime is None")
            return None

        # get q_old and q_next
        old_loc = tuple(self.states[0]) + (0, self.actions[0])
        q_old = self.q_table[old_loc]  # estimate

        action_prime = np.argmax(self.get_q_values(state_prime))
        next_loc = tuple(state_prime) + (0, action_prime)
        q_next = self.q_table[next_loc]  # max a' Q(state_{lamda}, a')

        truth = 0
        for i in range(len(self.rewards)):
            truth += self.rewards[i] * (self.discount_rate ** i)

        truth += (self.discount_rate ** self.lamda) * q_next  # truth
        error = truth - q_old

        # Q Table
        self.q_table[old_loc] = q_old + self.learning_rate * error
        return float(error)

    def get_reward(self, scan_data, vel, car, wp, is_collision):
        """
        car: [(x,y), yaw]
        wp: [(x,y), yaw]
        """
        if is_collision:
            return -20
        else:
            dist_error = math.sqrt((car[0][0] - wp[0][0]) ** 2 + (car[0][1] - wp[0][1]) ** 2)
            # assign error if facing away from waypoint
            angle_error = abs(car[1] - wp[1]) / math.pi
            return 10*(1 - dist_error) #  - angle_error)

    def action_to_ackermann(self, action):
        # returns (steering_angle, speed)
        return self.action_map[action]

    def ackermann_to_action(self, ackermann):
        # returns closest action
        return min(self.action_map, key=lambda x: abs(self.action_map[x][0] - ackermann.drive.steering_angle) + abs(self.action_map[x][1] - ackermann.drive.speed))


    def clamp_state(self, ranges):
        # round to nearest 0.2 and clamp between 0-4 so we get [0.2 ... 1.0] as [0 ... 4]
        for i in range(len(ranges)):
            ranges[i] = int( min( max( round(ranges[i] * 5), 0 ), (self.q_table.shape[i] -1) ) )
        #ranges[2] = int(min(max(round(ranges[2] * 2) / 2, 0), 19) * 2)
        return ranges
