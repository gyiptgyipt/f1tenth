import math

class PurePursuit:
    def get_control(self, car_length, alpha, distance):
        # pure pursuit calculation
        return -math.atan(2 * car_length * math.sin(alpha) / distance)

