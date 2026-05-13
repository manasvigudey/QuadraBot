import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from config.robot_config import *

#ik function
def leg_ik(px, pz):
    distance = np.sqrt(px**2 + pz**2)
    
    cos_knee = (L_THIGH**2 + L_CALF**2 - distance**2) / (2 * L_THIGH * L_CALF)
    cos_knee = np.clip(cos_knee, -1, 1)
    knee_angle = -np.arccos(cos_knee)
    
    alpha = np.arctan2(px, pz)
    beta = np.arccos(np.clip((L_THIGH**2 + distance**2 - L_CALF**2) / (2 * L_THIGH * distance), -1, 1))
    thigh_angle = alpha - beta
    
    return thigh_angle, knee_angle

#gait function
def swing_trajectory(t):
    angle = t * np.pi
    x = (STEP_LENGTH / 2) * np.cos(angle)
    z = STEP_HEIGHT * np.sin(angle)
    return x, z

def foot_position(phase):
    if phase < 0.5:
        t = phase / 0.5
        x, z_lift = swing_trajectory(t)
        z = STAND_DEPTH - z_lift
    else:
        t = (phase - 0.5) / 0.5
        x = (STEP_LENGTH / 2) - (STEP_LENGTH * t)
        z = STAND_DEPTH
    return x, z

def trot(leg, time):
    phase = (time / PERIOD + PHASE_OFFSET[leg]) % 1.0
    return foot_position(phase)

# Connect gait → IK for all 4 legs over one full trot cycle
print(f"{'Time':<6} {'Leg':<5} {'Foot x':>8} {'Foot z':>8} {'Thigh°':>8} {'Knee°':>8}")
print("-" * 50)

for i in range(11):
    t = i * 0.06
    for leg in ["FR", "FL", "RR", "RL"]:
        x, z = trot(leg, t)
        thigh, knee = leg_ik(x, z)
        print(f"t={t:.2f}  {leg}   x={x:>6.3f}  z={z:>6.3f}  {np.degrees(thigh):>7.1f}°  {np.degrees(knee):>7.1f}°")
    print()