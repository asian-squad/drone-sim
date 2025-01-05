import math
import pybullet as p
from .orientation import OrientationController

class QuadcopterController(OrientationController):

    def __init__(self, thrust: float, kp: float, ki: float, kd: float):
        self.thrust = thrust
        self.starting_thrust = thrust
        super().__init__(kp, ki, kd)

    def get_thrusts(self, roll: float, pitch: float, yaw: float, dt: float) -> tuple:
        control_roll, control_pitch, control_yaw = self.update(roll, pitch, yaw, dt)

        t_fl = self.thrust + control_yaw + control_roll - control_pitch
        t_fr = self.thrust - control_yaw - control_roll - control_pitch
        t_bl = self.thrust - control_yaw + control_roll + control_pitch
        t_br = self.thrust + control_yaw - control_roll + control_pitch

        return t_fl, t_fr, t_bl, t_br
    

    def keyboard(self, keys, dt: float):
        if p.B3G_SPACE in keys:
            self.thrust += 0.1 * dt
        if p.B3G_SHIFT in keys:
            self.thrust -= 0.1 * dt

        if p.B3G_LEFT_ARROW in keys:
            self.target_yaw += 0.5 * dt
            if self.target_yaw >= math.pi:
                self.target_yaw = -math.pi
        if p.B3G_RIGHT_ARROW in keys:
            self.target_yaw -= 0.5 * dt
            if self.target_yaw <= -math.pi:
                self.target_yaw = math.pi
        if p.B3G_UP_ARROW in keys:
            self.target_pitch += 0.2 * dt
            # if self.target_pitch >= math.pi:
                # self.target_pitch = -math.pi
        elif p.B3G_DOWN_ARROW in keys:
            self.target_pitch -= 0.2 * dt
            # if self.target_pitch <= -math.pi:
                # self.target_pitch = math.pi
        else:
            self.target_pitch = 0

        if p.B3G_CONTROL in keys:
            self.target_roll -= 0.2 * dt
            # if self.target_roll <= -math.pi:
                # self.target_roll = math.pi
        elif p.B3G_ALT in keys:
            self.target_roll += 0.2 * dt
            # if self.target_roll >= math.pi:
                # self.target_roll = -math.pi
        else:
            self.target_roll = 0
        
        if p.B3G_RETURN in keys:
            self.target_roll, self.target_pitch, self.target_yaw = 0, 0, 0
            self.thrust = self.starting_thrust


        
