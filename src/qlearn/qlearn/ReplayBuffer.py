
import numpy as np
import random

class ReplayBuffer:
    def __init__(self, buffer_capacity=10000):
        self.buffer_capacity = buffer_capacity
        self.buffer = []
        self.buffer_index = 0

    def add(self, state, action_st, action_sp, reward, state_prime, done):
        if len(self.buffer) < self.buffer_capacity:
            self.buffer.append([state, action_st, action_sp, reward, state_prime, done ])
        else:  # overwrite
            self.buffer[self.buffer_index] = [state, action_st, action_sp, reward, state_prime, done]
        self.buffer_index = (self.buffer_index + 1) % self.buffer_capacity

    def sample(self, random_size=32, recent_size=16):
        if len(self.buffer) < random_size + recent_size:
            return None
        else:
            # get random indices that include state_prime and done
            # make first 5 indices the most recent ones (using buffer_index)
            indices = [ (self.buffer_index-i-1) % self.buffer_capacity for i in range(recent_size) ]
            for i in range(random_size):
                indices.append(self.get_random_index())

            states = np.array([self.buffer[i][0] for i in indices])
            actions_st = np.array([self.buffer[i][1] for i in indices])
            actions_sp = np.array([self.buffer[i][2] for i in indices])
            rewards = np.array([self.buffer[i][3] for i in indices])
            states_prime = np.array([self.buffer[i][4] for i in indices])
            dones = np.array([self.buffer[i][5] for i in indices])

            return states, actions_st, actions_sp, rewards, states_prime, dones

    def get_random_index(self):
        return random.randint(0, len(self.buffer)-1)

    def size(self):
        return len(self.buffer)

    def capacity(self):
        return self.buffer_capacity

    def clear(self):
        self.buffer = np.array([])
        self.buffer_index = 0
