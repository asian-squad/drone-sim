from realistic_quadcopters import RealisticQuadcopter
from quadcopter import Quadcopter
from params import GRAVITY, TIME_STEP
import pybullet as p
import time
from pid_controller import PIDController

p.connect(p.GUI)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(TIME_STEP)

drone = RealisticQuadcopter(0.25, 0.3, 0.1, 1)

KP = 0.1
KI = 0.005
KD = 0.001

yaw_pid = PIDController(KP, KI, KD)
roll_pid = PIDController(KP, KI, KD)
pitch_pid = PIDController(KP, KI, KD)

T = -GRAVITY / 4

roll = 0
pitch = 0
yaw = 0

target_yaw = 0
target_pitch = 0
target_roll = 0
extra_thrust = 0

def follow(body: int):
    basePos, _ = p.getBasePositionAndOrientation(body)
    p.resetDebugVisualizerCamera(
        cameraDistance=3, cameraYaw=-90, cameraPitch=-30, cameraTargetPosition=basePos)

while True:
    acc, gyro = drone.get_acc_and_gyro(TIME_STEP)

    roll += gyro[0] * TIME_STEP
    pitch += gyro[1] * TIME_STEP
    yaw += gyro[2] * TIME_STEP

    roll_control = roll_pid.update(target_roll, roll, TIME_STEP)
    pitch_control = pitch_pid.update(target_pitch, pitch, TIME_STEP)
    yaw_control = yaw_pid.update(target_yaw, yaw, TIME_STEP)

    T_FL = T + yaw_control + roll_control - pitch_control + extra_thrust
    T_FR = T - yaw_control - roll_control - pitch_control + extra_thrust
    T_BL = T - yaw_control + roll_control + pitch_control + extra_thrust
    T_BR = T + yaw_control - roll_control + pitch_control + extra_thrust

    # keyboard control
    keys = p.getKeyboardEvents()
    if p.B3G_LEFT_ARROW in keys:
        target_yaw += 0.5 * TIME_STEP

    if p.B3G_RIGHT_ARROW in keys:
        target_yaw -= 0.5 * TIME_STEP

    if p.B3G_UP_ARROW in keys:
        target_pitch = 0.2
    elif p.B3G_DOWN_ARROW in keys:
        target_pitch = -0.2
    else:
        target_pitch = 0

    if p.B3G_SPACE in keys:
        extra_thrust = 0.2
    elif p.B3G_SHIFT in keys:
        extra_thrust = -0.2
    else:
        extra_thrust = 0

    follow(drone.body)

    drone.apply_thrust(T_FL, T_FR, T_BL, T_BR)

    print(f"roll_control: {float(roll_control)} roll: {
          roll}, pitch: {pitch}, yaw: {yaw}")

    p.stepSimulation()
    time.sleep(TIME_STEP)
