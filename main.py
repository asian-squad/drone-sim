from noisy_quadcopter import NoisyQuadcopter
from quadcopter import Quadcopter
from params import GRAVITY, TIME_STEP
import pybullet as p
import time
from pid_controller import PIDController

p.connect(p.GUI)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(TIME_STEP)

drone = NoisyQuadcopter(0.25, 0.3, 0.1, 1)

KP = 3
KI = 5.5
KD = 4

yaw_pid = PIDController(KP, KI, KD)
roll_pid = PIDController(KP, KI, KD)
pitch_pid = PIDController(KP, KI, KD)

T = -GRAVITY / 4

roll = 0
pitch = 0
yaw = 0

while True:
    acc, gyro = drone.get_acc_and_gyro(TIME_STEP)

    roll += gyro[0] * TIME_STEP
    pitch += gyro[1] * TIME_STEP
    yaw += gyro[2] * TIME_STEP

    RATE = 0.001
    # roll_control = roll_pid.update(0, roll, TIME_STEP) * RATE
    roll_control = -0.0005
    # pitch_control = pitch_pid.update(0, pitch, TIME_STEP) * RATE
    # yaw_control = yaw_pid.update(0, yaw, TIME_STEP) * RATE

    # T_FL = T + yaw_control + roll_control + pitch_control
    # T_FR = T - yaw_control - roll_control + pitch_control
    # T_BL = T - yaw_control + roll_control - pitch_control
    # T_BR = T + yaw_control - roll_control - pitch_control

    T_FL = T + roll_control
    T_FR = T - roll_control
    T_BL = T + roll_control
    T_BR = T - roll_control

    drone.apply_thrust(T_FL, T_FR, T_BL, T_BR)

    # print(f"roll: {gyro[0]}, pitch: {gyro[1]}, yaw: {gyro[2]}")
    print(f"roll_control: {float(roll_control)} roll: {roll}, pitch: {pitch}, yaw: {yaw}")
    # drone.apply_thrust(T * 0.9, T * 0.9, T * 1.1, T * 1.1)
    # drone.apply_thrust(T, T, T, T)

    basePos, baseOrn = p.getBasePositionAndOrientation(drone.body)
    # p.resetDebugVisualizerCamera( cameraDistance=1, cameraYaw=90, cameraPitch=0, cameraTargetPosition=basePos)

    p.stepSimulation()
    time.sleep(TIME_STEP)
