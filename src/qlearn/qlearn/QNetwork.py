
import torch
import torch.nn as nn
import torch.nn.functional as F

import numpy as np
import random
import math
import copy

from .ReplayBuffer import ReplayBuffer
from .DQN import DQN


class QNetwork:
    def __init__(self, learning_rate=0.01, discount_rate=0.9, random_rate=0.2, top_speed=15.0):

        self.discount_rate = discount_rate
        self.learning_rate = learning_rate
        self.random_rate = random_rate

        self.replay_buffer = ReplayBuffer(buffer_capacity=2000)
        self.buffer_warmup_ratio = 0.25

        self.input_dim = 17
        self.steering_angles = self.generate_halving_array(0.4, 0.0125)
        self.speeds = [float(x/6.0) for x in range(-1, 2, 1)]  # actually throttle, #2 is non-inclusive
        self.top_speed = top_speed

        print("Top speed:", self.top_speed)
        print("Steering angles:", self.steering_angles)
        print("Speeds:", self.speeds)

        if torch.cuda.is_available():  
            dev = "cuda:0" 
        else:  
            dev = "cpu"  
        self.device = torch.device(dev)  

        self.model = DQN(self.input_dim, len(self.steering_angles), len(self.speeds)).to(self.device)
        self.target_model = copy.deepcopy(self.model).to(self.device)
        self.target_model.eval()

        self.epochs = 1
        self.loss_steering = nn.SmoothL1Loss()
        self.loss_speed = nn.SmoothL1Loss()

        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)
        self.scheduler1 = torch.optim.lr_scheduler.ReduceLROnPlateau(self.optimizer, patience=100, factor=0.8, verbose=True, min_lr=0.0001)
        #self.scheduler2 = torch.optim.lr_scheduler.ExponentialLR(self.optimizer, gamma=0.996)

        self.update_random_size = 32
        self.update_recent_size = 0

        self.target_counter = 0
        self.target_update_freq = 100  # 25 or 100


    def get_state(self, scan_data, vel, steering_angle):
        # 20 rays ~ 5 degrees; should be size self.input_dim
        return self.clamp_state( np.array([ 
            vel,
            steering_angle,
            scan_data.ranges[540+360],  # left
            scan_data.ranges[540+180], 
            scan_data.ranges[540+135],
            scan_data.ranges[540+90], 
            scan_data.ranges[540+45],
            scan_data.ranges[540+30],
            scan_data.ranges[540+15],
            scan_data.ranges[540],  # center
            scan_data.ranges[540-15],
            scan_data.ranges[540-30],
            scan_data.ranges[540-45],
            scan_data.ranges[540-90], 
            scan_data.ranges[540-135],
            scan_data.ranges[540-180],
            scan_data.ranges[540-360],  # right
        ]) )

    def get_q_values(self, state, m = None):
        if m is None:
            m = self.model
        state_prime = torch.tensor(state).float().to(self.device)
        with torch.no_grad():
            q_values_st, q_values_sp = m(state_prime)
        return q_values_st.cpu().numpy(), q_values_sp.cpu().numpy()

    def get_action_index(self, q_values_st, q_values_sp, use_randomness=True):
        if use_randomness and random.random() < self.random_rate:
            return random.randint(0, len(self.steering_angles)-1), random.randint(0, len(self.speeds)-1)
        else:
            return [np.argmax(q_values_st), np.argmax(q_values_sp)]

    def get_action_ack(self, idx, vel, top_speed, min_speed):
        new_vel = vel + self.speeds[idx[1]]
        new_vel = min(top_speed, max(min_speed, new_vel))
        return [self.steering_angles[idx[0]], new_vel]  # acceleration

    def update(self):

        total_loss_st = 0
        total_loss_sp = 0
        sample_size = self.update_random_size + self.update_recent_size
        if self.replay_buffer.size() < self.replay_buffer.capacity() * self.buffer_warmup_ratio:
            return None, None  # fill buffer first

        for _ in range(self.epochs):
            states, st_actions, sp_actions, rewards, states_prime, dones = self.replay_buffer.sample(self.update_random_size, self.update_recent_size)  # batch of (state, action, reward, state_prime, done)

            q_estimates_st, q_estimates_sp = self.model(torch.tensor(states).float().to(self.device))
            q_estimates_st = q_estimates_st[np.arange(sample_size), st_actions.astype(int)]
            q_estimates_sp = q_estimates_sp[np.arange(sample_size), sp_actions.astype(int)]

            # get q_truth. Only use reward if done
            q_truths_st = rewards.copy()
            for i in range(sample_size):
                if not dones[i]:
                    q_truths_st[i] += self.discount_rate * np.max(self.get_q_values(states_prime[i], self.target_model)[0])

            q_truths_sp = rewards.copy()
            for i in range(sample_size):
                if not dones[i]:
                    q_truths_sp[i] += self.discount_rate * np.max(self.get_q_values(states_prime[i], self.target_model)[1])

            # Neural Network
            loss_st = self.loss_steering(q_estimates_st, torch.tensor(q_truths_st).float().to(self.device))
            loss_sp = self.loss_speed(q_estimates_sp, torch.tensor(q_truths_sp).float().to(self.device))
            self.optimizer.zero_grad()
            loss_st.backward()
            loss_sp.backward()

            #torch.nn.utils.clip_grad_norm_(self.model.parameters(), 100.0)
            self.optimizer.step()
            
            total_loss_st += loss_st.item()
            total_loss_sp += loss_sp.item()

        self.target_counter += 1
        if self.target_counter >= self.target_update_freq:
            self.target_counter = 0
            self.target_model.load_state_dict(self.model.state_dict())

        avg_loss_st = total_loss_st / self.epochs
        avg_loss_sp = total_loss_sp / self.epochs
        avg_loss = avg_loss_st + avg_loss_sp
        self.scheduler1.step(avg_loss)
        #self.scheduler2.step()

        return avg_loss_st, avg_loss_sp

    def get_reward(self, vel, car, last_car, wp, is_collision):
        """
        car: [(x,y), yaw, speed]
        wp: [(x,y), yaw]
        """
        reward = 0
        if is_collision:
            reward -= 20
        else:
            reward += 2
            reward -= math.sqrt((car[0][0] - wp[0][0]) ** 2 + (car[0][1] - wp[0][1]) ** 2)  # distance error
            reward += vel / (self.top_speed)  # speed
            #reward -= abs(car[2] - last_car[2])  # speed derivative
            #reward -= abs(car[1] - last_car[1])  # yaw derivative
            #reward -= abs(car[1] - wp[1])  # angle error
        return reward / 20

    def ackermann_to_action_index(self, ackermann):
        # returns closest action
        steering = ackermann.steering_angle
        speed = ackermann.speed
        steering_index = np.argmin([abs(steering - x) for x in self.steering_angles])
        speed_index = np.argmin([abs(speed - x) for x in self.speeds])
        return [steering_index, speed_index]

    def set_learning_rate(self, learning_rate):
        self.learning_rate = learning_rate
        for param_group in self.optimizer.param_groups:
            param_group['lr'] = learning_rate

    def generate_halving_array(self, max_value, min_value):
        array = [float(max_value)]
        while array[-1] / 2 >= min_value:
            array.append(float(array[-1] / 2))
        negative_part = [float(-x) for x in array[::-1]]
        array += [0.0] + negative_part  # combine
        return array

    def clamp_state(self, state):
        #ranges = state.copy()
        #for i in range(len(ranges)):
        #    ranges[i] = int( min( max( round(ranges[i] * 5), 0 ), 5 * 10 ) )  / 5 * 10
        ranges = np.clip(state, 0.0, 20.0) / 20.0
        return ranges
