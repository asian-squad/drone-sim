"""
Kd = 4;
Kp = 3;
Ki = 5.5;
"""

class PIDController:
    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.prev_error = 0
        self.error_integral = 0

    def update(self, x_set: float, x: float, dt: float) -> float:
        error = x_set - x

        self.error_integral += error * dt
        error_derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.k_p * error + self.k_i * self.error_integral + self.k_d * error_derivative