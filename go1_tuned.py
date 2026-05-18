"""
GO1 Improved — Tuned Trot Simulation
Improvements over go1_simulation.py:
1. Slower period (0.6 → 0.7s) — more stable
2. Smaller steps (0.08 → 0.06m) — less wobble  
3. Lower step height (0.055 → 0.045m) — smoother
4. Higher position gains — more precise joint control
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import sys

L_HIP   = 0.08
L_THIGH = 0.213
L_CALF  = 0.255
STAND_HEIGHT = 0.27

LEG_NAMES = ["FR", "FL", "RR", "RL"]

HIP_OFFSETS = {
    "FR": ( 0.1881, -0.04675),
    "FL": ( 0.1881,  0.04675),
    "RR": (-0.1881, -0.04675),
    "RL": (-0.1881,  0.04675),
}

def leg_ik(foot_pos_body, hip_offset_y):
    px, py, pz = foot_pos_body
    dy = py - hip_offset_y
    dz = pz
    hip_ab = np.arctan2(dy, dz)
    lat_dist = np.sqrt(dy**2 + dz**2) - L_HIP
    l = np.sqrt(px**2 + lat_dist**2)
    if l > (L_THIGH + L_CALF) or l < abs(L_THIGH - L_CALF):
        return None
    cos_calf = (L_THIGH**2 + L_CALF**2 - l**2) / (2 * L_THIGH * L_CALF)
    cos_calf = np.clip(cos_calf, -1, 1)
    calf = -(np.pi - np.arccos(cos_calf))
    alpha = np.arctan2(px, lat_dist)
    beta  = np.arccos(np.clip((L_THIGH**2 + l**2 - L_CALF**2) /
                               (2 * L_THIGH * l), -1, 1))
    thigh = alpha - beta
    return hip_ab, thigh, calf

class TrotGait:
    def __init__(self):
        self.step_length = 0.06
        self.step_height = 0.045
        self.period      = 0.7
        self.body_height = STAND_HEIGHT
        self.phase_offset = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}

    def foot_position(self, leg, t):
        phase = (t / self.period + self.phase_offset[leg]) % 1.0
        hip_y = HIP_OFFSETS[leg][1]
        if phase < 0.5:
            sw = phase / 0.5
            x = (self.step_length / 2) * np.cos(np.pi * sw)
            z = self.body_height - self.step_height * np.sin(np.pi * sw)
        else:
            st = (phase - 0.5) / 0.5
            x = -(self.step_length / 2) * np.cos(np.pi * st)
            z = self.body_height
        y = hip_y
        return [x, y, z]

def run_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    p.resetDebugVisualizerCamera(1.2, 45, -25, [0, 0, 0])
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    ground = p.loadURDF("plane.urdf")
    p.changeDynamics(ground, -1, lateralFriction=0.8)

    robot = p.loadURDF(
        "go1_improved_nomesh.urdf",
        [0, 0, 0.35],
        p.getQuaternionFromEuler([3.14159, 0, 0]),
        flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
    )

    joint_map = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        joint_map[info[1].decode("utf-8")] = i

    leg_joints = {}
    for leg in LEG_NAMES:
        leg_joints[leg] = [
            joint_map[f"{leg}_hip_joint"],
            joint_map[f"{leg}_thigh_joint"],
            joint_map[f"{leg}_calf_joint"]
        ]

    print("Settling...")
    for _ in range(500):
        for leg, ids in leg_joints.items():
            for jid, angle in zip(ids, [0.0, 0.8, -1.6]):
                p.setJointMotorControl2(robot, jid,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle, force=35,
                    positionGain=0.5, velocityGain=0.05)
        p.stepSimulation()

    print("Walking (tuned)...")
    gait = TrotGait()
    sim_time = 0.0
    dt = 1.0 / 500.0

    while p.isConnected():
        keys = p.getKeyboardEvents()
        if ord('q') in keys or 27 in keys:
            break

        for leg, ids in leg_joints.items():
            foot_pos = gait.foot_position(leg, sim_time)
            result = leg_ik(foot_pos, HIP_OFFSETS[leg][1])
            if result is None:
                continue
            hip_ab, thigh, calf = result
            for jid, angle, force, pg, dg in zip(
                    ids, [hip_ab, thigh, calf],
                    [23.7, 23.7, 35.55],
                    [0.8, 1.0, 1.0],
                    [0.05, 0.05, 0.05]):
                p.setJointMotorControl2(robot, jid,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle, force=force,
                    positionGain=pg, velocityGain=dg)

        base_pos, _ = p.getBasePositionAndOrientation(robot)
        p.resetDebugVisualizerCamera(1.2, 45, -25, base_pos)
        p.stepSimulation()
        sim_time += dt
        time.sleep(dt)

    p.disconnect()
    print("Done!")

if __name__ == "__main__":
    run_simulation()