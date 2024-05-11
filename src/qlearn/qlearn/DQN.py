import torch
import torch.nn as nn

class DQN(nn.Module):
    def __init__(self, input_dim, num_steering_actions, num_velocity_actions):
        super(DQN, self).__init__()

        self.input_dim = input_dim
        self.num_steering_actions = num_steering_actions
        self.num_velocity_actions = num_velocity_actions

        #self.layer1 = nn.Sequential(nn.Linear(input_dim, 512), nn.ReLU())

        self.steer_branch = nn.Linear(input_dim, 512)
        self.velocity_branch = nn.Linear(input_dim, 128)

        self.steer_layer = nn.Sequential(
            nn.ReLU(),
            nn.Linear(512, 512), nn.ReLU(),
            nn.Linear(512, num_steering_actions),
        )
        self.velocity_layer = nn.Sequential(
            nn.ReLU(),
            nn.Linear(128, 128), nn.ReLU(),
            nn.Linear(128, num_velocity_actions),
        )

    def forward(self, x):
        # Shared layers
        #x = self.shared_layers(x)

        # Branching
        a = self.steer_branch(x)
        steering_q_values = self.steer_layer(a)

        b = self.velocity_branch(x)
        velocity_q_values = self.velocity_layer(b)

        return steering_q_values, velocity_q_values
