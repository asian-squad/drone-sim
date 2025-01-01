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
controller = QuadcopterController(-GRAVITY / 4, 0.2, 0.2, 0.01)

def to_deg(rad: float) -> int:
    return int(rad * 180 / math.pi)

def follow(body: int, yaw: float):
    basePos, _ = p.getBasePositionAndOrientation(body)
    p.resetDebugVisualizerCamera(
        cameraDistance=3, cameraYaw=to_deg(yaw), cameraPitch=-30, cameraTargetPosition=basePos)

while True:
    acc, gyro, orn = drone.get_acc_gyro_orn(TIME_STEP)

    roll = orn[0]
    pitch = orn[1]
    yaw = orn[2]
    
    drone.apply_thrusts(*controller.get_thrusts(roll, pitch, yaw, TIME_STEP))
    controller.keyboard(TIME_STEP)

    follow(drone.body, yaw - math.pi / 2)
    p.stepSimulation()
    time.sleep(TIME_STEP)
