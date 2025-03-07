from realistic_quadcopters import RealisticQuadcopter
from quadcopter import Quadcopter
from params import GRAVITY, TIME_STEP
import pybullet as p
import time
from controller.quadcopter import QuadcopterController
import math
import pybullet_data

p.connect(p.GUI)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(TIME_STEP)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

drone = RealisticQuadcopter(0.25, 0.3, 0.1, 1)
drone.set_position(0, 0, 1)
controller = QuadcopterController(-GRAVITY / 4, 0.2, 0.2, 0.1)

def to_degrees(rad: float) -> int:
    return int(rad * 180 / math.pi)

def follow(body: int, yaw: float):
    basePos, _ = p.getBasePositionAndOrientation(body)
    p.resetDebugVisualizerCamera(
        cameraDistance=3, cameraYaw=to_degrees(yaw), cameraPitch=-30, cameraTargetPosition=basePos)

while True:
    acceleration, gyro, orientation = drone.get_acceleration_gyro_orientation(TIME_STEP)

    roll = orientation[0]
    pitch = orientation[1]
    yaw_velocity = gyro[2] / 10
    
    drone.apply_thrusts(*controller.get_thrusts(roll, pitch, yaw_velocity, TIME_STEP))

    keys = p.getKeyboardEvents()
    controller.keyboard(keys, TIME_STEP)

    if p.B3G_RETURN in keys:
        drone.set_position(0, 0, 1)
        
    follow(drone.body, yaw_velocity - math.pi / 2)
    p.stepSimulation()
    time.sleep(TIME_STEP)
