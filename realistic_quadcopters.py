from params import MAX_THRUST
from quadcopter import Quadcopter
import numpy as np
import pybullet as p


class RealisticQuadcopter(Quadcopter):
    def __init__(self, width, length, height, mass, x_center_of_mass=0.0, y_center_of_mass=0.0, z_center_of_mass=0.0, propeller_diameter=0.12, propeller_height=0.01, noise_standard_deviation=0.001):
        super().__init__(width, length, height, mass, x_center_of_mass, y_center_of_mass, z_center_of_mass, propeller_diameter, propeller_height)
        self.noise_std = noise_standard_deviation
        p.changeDynamics(self.body, -1, linearDamping=0.1, angularDamping=0.1)

    def apply_thrusts(self, front_left, front_right, back_left, back_right):
        front_left += np.random.normal(0, self.noise_std)
        front_right += np.random.normal(0, self.noise_std)
        back_left += np.random.normal(0, self.noise_std)
        back_right += np.random.normal(0, self.noise_std)

        front_left = np.clip(front_left, 0, MAX_THRUST)
        front_right = np.clip(front_right, 0, MAX_THRUST)
        back_left = np.clip(back_left, 0, MAX_THRUST)
        back_right = np.clip(back_right, 0, MAX_THRUST)

        super().apply_thrusts(front_left, front_right, back_left, back_right)