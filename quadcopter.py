from quadcopter_urdf import quadcopter_urdf
from params import GRAVITY, THRUST_TO_TORQUE
import pybullet as p
import numpy as np


class Quadcopter:
    mass: float
    body: int

    thrust_positions: list[tuple[float, float, float]]
    thurst_directions: tuple[int, int, int, int]
    prev_velocity: np.ndarray

    def __init__(self, w, l, h, mass, x_com=0.0, y_com=0.0, z_com=0.0, d_prop=0.12, h_prop=0.01):

        self.mass = mass
        self.body = p.loadURDF(quadcopter_urdf(
            w, l, h, mass, x_com, y_com, z_com, d_prop, h_prop))
        
        p.changeDynamics(self.body, -1, linearDamping=0, angularDamping=0)

        half_w = w / 2
        half_l = l / 2

        fl_pos = [half_l, half_w, 0]
        fr_pos = [half_l, -half_w, 0]
        bl_pos = [-half_l, half_w, 0]
        br_pos = [-half_l, -half_w, 0]

        self.thrust_positions = [fl_pos, fr_pos, bl_pos, br_pos]
        self.thurst_directions = [1, -1, -1, 1]
        self.prev_velocity = np.zeros(3)

    def apply_thrust(self, front_left, front_right, back_left, back_right):
        thrusts = [front_left, front_right, back_left, back_right]

        for force, pos, direction in zip(thrusts, self.thrust_positions, self.thurst_directions):
            p.applyExternalForce(
                self.body, -1, forceObj=[0, 0, force], posObj=pos, flags=p.LINK_FRAME)

            torque = force * THRUST_TO_TORQUE * direction

            p.applyExternalTorque(
                self.body, -1, torqueObj=[0, 0, torque], flags=p.LINK_FRAME)

    def get_acc_and_gyro(self, dt):

        link_state = p.getLinkState(self.body, 0, computeLinkVelocity=1)

        rotation_matrix = p.getMatrixFromQuaternion(link_state[1])
        rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
        linear_velocity = np.array(link_state[6])
        angular_velocity = np.array(link_state[7])

        local_linear_velocity = np.dot(rotation_matrix.T, linear_velocity)
        local_angular_velocity = np.dot(rotation_matrix.T, angular_velocity)

        acceleration = (local_linear_velocity - self.prev_velocity) / dt
        self.prev_velocity = local_linear_velocity
        acceleration[2] -= GRAVITY

        return acceleration, local_angular_velocity
