from .pid import PIDController

KP = 0.2
KI = 0.2
KD = 0.01


class OrientationController:
    target_yaw = 0
    target_pitch = 0
    target_roll = 0
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.yaw_controller = PIDController(kp, ki, kd)
        self.roll_controller = PIDController(kp, ki, kd)
        self.pitch_controller = PIDController(kp, ki, kd)

    def update(self, roll: float, pitch: float, yaw: float, dt: float):
        error_roll = self.target_roll - roll
        error_pitch = self.target_pitch - pitch
        error_yaw = self.target_yaw - yaw

        control_roll = self.roll_controller.update(error_roll, dt)
        control_pitch = self.pitch_controller.update(error_pitch, dt)
        control_yaw = self.yaw_controller.update(error_yaw, dt)
        return control_roll, control_pitch, control_yaw
