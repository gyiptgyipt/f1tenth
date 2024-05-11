import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, initializers, losses, optimizers
import os

import rclpy
import math
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from my_messages.msg import PIDInput
from std_msgs.msg import Float32
from std_msgs.msg import Bool


class DQN():
    def __init__(self, state_size, num_actions, learning_rate=0.001, gamma=0.99):

        self.state_size = state_size
        self.num_actions = num_actions
        self.learning_rate = learning_rate
        self.gamma = gamma

        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_model()

    def build_model(self):
        model = tf.keras.Sequential([
            layers.Dense(64, activation='relu', input_shape=(self.state_size,), kernel_initializer=initializers.VarianceScaling(scale=2.)),
            layers.Dense(32, activation='relu', kernel_initializer=initializers.VarianceScaling(scale=2.)),
            layers.Dense(self.num_actions, activation='linear', kernel_initializer=initializers.VarianceScaling(scale=2.))
        ])

        model.compile(optimizer=optimizers.Adam(self.learning_rate),
                      loss=losses.Huber())
        return model
    
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    def select_action(self, state):
        state = np.reshape(state, (1, self.state_size))
        q_values = self.model.predict(state)
        action = np.argmax(q_values)
        print(action)
        return action
    
    def train(self, state, action, reward, next_state, done):
        state = np.reshape(state, (1, self.state_size))
        next_state = np.reshape(next_state, (1, self.state_size))

        target = self.model.predict(state)
        if done:
            target[0][action] = reward
        else:
            q_future = max(self.target_model.predict(next_state)[0])
            target[0][action] = reward + self.gamma * q_future
        
        self.model.fit(state, target, epochs=1, verbose=0)

        self.update_target_model()

