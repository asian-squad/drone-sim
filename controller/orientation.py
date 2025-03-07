from .pid import PIDController

class OrientationController:
    target_roll = 0
    target_pitch = 0
    target_yaw_velocity = 0 
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.roll_controller = PIDController(kp, ki, kd)
        self.pitch_controller = PIDController(kp, ki, kd)
        self.yaw_velocity_controller = PIDController(kp, ki, kd)

    def update(self, roll: float, pitch: float, yaw_velocity: float, dt: float):
        error_roll = self.target_roll - roll
        error_pitch = self.target_pitch - pitch
        error_yaw_velocity = self.target_yaw_velocity - yaw_velocity

        control_roll = self.roll_controller.update(error_roll, dt)
        control_pitch = self.pitch_controller.update(error_pitch, dt)
        control_yaw_velocity = self.yaw_velocity_controller.update(error_yaw_velocity, dt)
        return control_roll, control_pitch, control_yaw_velocity
