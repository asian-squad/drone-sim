from params import MAX_THRUST
from quadcopter import Quadcopter
import numpy as np


class RealisticQuadcopter(Quadcopter):
    def __init__(self, w, l, h, mass, x_com=0.0, y_com=0.0, z_com=0.0, d_prop=0.12, h_prop=0.01, noise_std=0.001):
        super().__init__(w, l, h, mass, x_com, y_com, z_com, d_prop, h_prop)
        self.noise_std = noise_std

    def apply_thrust(self, front_left, front_right, back_left, back_right):
        front_left += np.random.normal(0, self.noise_std)
        front_right += np.random.normal(0, self.noise_std)
        back_left += np.random.normal(0, self.noise_std)
        back_right += np.random.normal(0, self.noise_std)

        front_left = np.clip(front_left, 0, MAX_THRUST)
        front_right = np.clip(front_right, 0, MAX_THRUST)
        back_left = np.clip(back_left, 0, MAX_THRUST)
        back_right = np.clip(back_right, 0, MAX_THRUST)

        super().apply_thrust(front_left, front_right, back_left, back_right)

    def get_acc_and_gyro(self, dt):
        acc, gyro = super().get_acc_and_gyro(dt)
        # acc += np.random.normal(0, self.noise_std, 3)
        # gyro += np.random.normal(0, self.noise_std, 3)
        return acc, gyro
