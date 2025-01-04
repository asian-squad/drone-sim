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

    def __init__(self, width, length, height, mass, x_center_of_mass=0.0, y_center_of_mass=0.0, z_center_of_mass=0.0, propeller_diameter=0.12, propeller_height=0.01):

        self.mass = mass
        self.body = p.loadURDF(quadcopter_urdf(
            width, length, height, mass, x_center_of_mass, y_center_of_mass, z_center_of_mass, propeller_diameter, propeller_height))

        p.changeDynamics(self.body, -1, linearDamping=0, angularDamping=0)

        half_width = width / 2
        half_length = length / 2

        fl_pos = [half_length, half_width, 0]
        fr_pos = [half_length, -half_width, 0]
        bl_pos = [-half_length, half_width, 0]
        br_pos = [-half_length, -half_width, 0]

        self.thrust_positions = [fl_pos, fr_pos, bl_pos, br_pos]
        self.thurst_directions = [1, -1, -1, 1]
        self.prev_velocity = np.zeros(3)

    def set_position(self, x: float, y: float, z: float):
        p.resetBasePositionAndOrientation(self.body, [x, y, z], [0, 0, 0, 1])

    def apply_thrusts(self, front_left, front_right, back_left, back_right):
        thrusts = [front_left, front_right, back_left, back_right]

        for force, pos, direction in zip(thrusts, self.thrust_positions, self.thurst_directions):
            p.applyExternalForce(
                self.body, -1, forceObj=[0, 0, force], posObj=pos, flags=p.LINK_FRAME)

            torque = force * THRUST_TO_TORQUE * direction

            p.applyExternalTorque(
                self.body, -1, torqueObj=[0, 0, torque], flags=p.LINK_FRAME)

    def get_acceleration_gyro_orientation(self, dt):

        link_state = p.getLinkState(self.body, 0, computeLinkVelocity=1)

        orientation = p.getEulerFromQuaternion(link_state[1])
        rotation_matrix = p.getMatrixFromQuaternion(link_state[1])
        rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
        linear_velocity = np.array(link_state[6])
        angular_velocity = np.array(link_state[7])

        local_linear_velocity = np.dot(rotation_matrix.T, linear_velocity)
        local_angular_velocity = np.dot(rotation_matrix.T, angular_velocity)

        acceleration = (local_linear_velocity - self.prev_velocity) / dt
        self.prev_velocity = local_linear_velocity
        acceleration[2] -= GRAVITY

        return acceleration, local_angular_velocity, orientation
