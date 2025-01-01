import pybullet as p
from .orientation import OrientationController

class QuadcopterController(OrientationController):

    def __init__(self, thrust: float, kp: float, ki: float, kd: float):
        self.thrust = thrust
        super().__init__(kp, ki, kd)

    def get_thrusts(self, roll: float, pitch: float, yaw: float, dt: float) -> tuple:
        control_roll, control_pitch, control_yaw = self.update(roll, pitch, yaw, dt)

        t_fl = self.thrust + control_yaw + control_roll - control_pitch
        t_fr = self.thrust - control_yaw - control_roll - control_pitch
        t_bl = self.thrust - control_yaw + control_roll + control_pitch
        t_br = self.thrust + control_yaw - control_roll + control_pitch

        return t_fl, t_fr, t_bl, t_br
    

    def keyboard(self, dt: float):
        keys = p.getKeyboardEvents()

        if p.B3G_SPACE in keys:
            self.thrust += 0.1 * dt
        if p.B3G_SHIFT in keys:
            self.thrust -= 0.1 * dt

        if p.B3G_LEFT_ARROW in keys:
            self.target_yaw += 0.5 * dt
        if p.B3G_RIGHT_ARROW in keys:
            self.target_yaw -= 0.5 * dt

        if p.B3G_UP_ARROW in keys:
            self.target_pitch += 0.2 * dt
        elif p.B3G_DOWN_ARROW in keys:
            self.target_pitch -= 0.2 * dt
        else:
            self.target_pitch = 0

        if p.B3G_CONTROL in keys:
            self.target_roll -= 0.2 * dt
        elif p.B3G_ALT in keys:
            self.target_roll += 0.2 * dt
        else:
            self.target_roll = 0


        
