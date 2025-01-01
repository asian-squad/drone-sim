class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.error_integral = 0

    def update(self, error: float, dt: float) -> float:
        self.error_integral += error * dt
        error_derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp * error + self.ki * self.error_integral + self.kd * error_derivative