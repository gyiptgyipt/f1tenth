
class PID:
    def __init__(self, kp = 1.0, ki = 0.0, kd = 0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self._integral = 0.0
        self._prev_error = None
        self._prev_derivative = None

    def update(self, error, dt = 0.01):

        if self._prev_error is None:
            self._prev_error = error

        derivative = (error - self._prev_error)

        self._integral += error
        self._prev_error = error

        kp = self.kp
        error_dampening_threshold = 0.05
        if abs(error) < error_dampening_threshold: # small error
            kp = abs(error)*(1/error_dampening_threshold) * self.kp

        # # Low-pass filter for derivative term (optional)
        # if self._prev_derivative is None:
        #     self._prev_derivative = derivative
        # 
        # derivative = 0.6 * derivative + 0.4 * self._prev_derivative
        # self._prev_derivative = derivative

        # PID
        adjustment = error * kp + derivative * self.kd + self._integral * self.ki

        # Standard form
        # Ti = 1.0  # time constant for integral
        # Td = 1.0  # time constant for derivative
        # adjustment = kp * (error + 1/Ti * self._integral + Td * derivative)


        # Prevent integral windup
        if adjustment >= 0.1 or adjustment <= -0.1:
            self._integral = 0.0

        return adjustment
