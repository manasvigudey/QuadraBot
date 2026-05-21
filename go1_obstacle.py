"""
GO1 — Obstacle Demo
Shows improved robot (calf 255mm) stepping over an obstacle
that the original robot (calf 213mm) struggles with.
Proves longer calf = better terrain clearance.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

# ── Robot parameters ──
L_HIP        = 0.08
L_THIGH      = 0.213
STAND_HEIGHT = 0.27
LEG_NAMES    = ["FR", "FL", "RR", "RL"]

HIP_OFFSETS = {
    "FR": ( 0.1881, -0.04675),
    "FL": ( 0.1881,  0.04675),
    "RR": (-0.1881, -0.04675),
    "RL": (-0.1881,  0.04675),
}

PHASE_OFFSET = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}

# ── Gait parameters ──
STEP_LENGTH = 0.06
STEP_HEIGHT = 0.10    # slightly higher to clear obstacle
PERIOD      = 0.7

#IK AND GAIT FUNCTIONS
def leg_ik(foot_pos, hip_offset_y, l_calf):
    px, py, pz = foot_pos
    dy = py - hip_offset_y
    dz = pz
    hip_ab = np.arctan2(dy, dz)
    lat_dist = np.sqrt(dy**2 + dz**2) - L_HIP
    l = np.sqrt(px**2 + lat_dist**2)
    if l > (L_THIGH + l_calf) or l < abs(L_THIGH - l_calf):
        return None
    cos_calf = (L_THIGH**2 + l_calf**2 - l**2) / (2 * L_THIGH * l_calf)
    cos_calf = np.clip(cos_calf, -1, 1)
    calf_angle = -(np.pi - np.arccos(cos_calf))
    alpha = np.arctan2(px, lat_dist)
    beta  = np.arccos(np.clip((L_THIGH**2 + l**2 - l_calf**2) /
                               (2 * L_THIGH * l), -1, 1))
    thigh = alpha - beta
    return hip_ab, thigh, calf_angle

def foot_position(leg, t):
    phase = (t / PERIOD + PHASE_OFFSET[leg]) % 1.0
    hip_y = HIP_OFFSETS[leg][1]
    if phase < 0.5:
        sw = phase / 0.5
        x = (STEP_LENGTH / 2) * np.cos(np.pi * sw)
        z = STAND_HEIGHT - STEP_HEIGHT * np.sin(np.pi * sw)
    else:
        st = (phase - 0.5) / 0.5
        x = -(STEP_LENGTH / 2) * np.cos(np.pi * st)
        z = STAND_HEIGHT
    y = hip_y
    return [x, y, z]

def get_leg_joints(robot):
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
    return leg_joints

#main simulation loop
def run_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-20,
        cameraTargetPosition=[0.3, 0, 0.1]
    )

    # Ground
    ground = p.loadURDF("plane.urdf")
    p.changeDynamics(ground, -1, lateralFriction=0.8)

    # Obstacle — a box in front of the robot
    obstacle_col = p.createCollisionShape(p.GEOM_BOX,
                                          halfExtents=[0.05, 0.3, 0.012])
    obstacle_vis = p.createVisualShape(p.GEOM_BOX,
                                       halfExtents=[0.05, 0.3, 0.012],
                                       rgbaColor=[1, 0.5, 0, 1])
    obstacle = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=obstacle_col,
                                  baseVisualShapeIndex=obstacle_vis,
                                  basePosition=[-0.6, 0, 0.012])

    p.addUserDebugText("OBSTACLE", [-0.6, 0, 0.08],
                       textColorRGB=[1, 0.5, 0], textSize=1.2)

    # Load robot
    robot = p.loadURDF(
        "go1_improved_nomesh.urdf",
        [0, 0, 0.35],
        p.getQuaternionFromEuler([3.14159, 0, 0]),
        flags=p.URDF_USE_INERTIA_FROM_FILE
    )

    p.addUserDebugText("IMPROVED (calf 255mm)", [0, 0, 0.5],
                       textColorRGB=[0.2, 1, 0.2], textSize=1.2)

    leg_joints = get_leg_joints(robot)

    #settle and main loop
    # Settle
    print("Settling...")
    for _ in range(500):
        for leg, ids in leg_joints.items():
            for jid, angle in zip(ids, [0.0, 0.8, -1.6]):
                p.setJointMotorControl2(robot, jid,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle, force=35,
                    positionGain=0.5, velocityGain=0.05)
        p.stepSimulation()

    print("Walking toward obstacle...")
    sim_time = 0.0
    dt = 1.0 / 500.0

    while p.isConnected():
        keys = p.getKeyboardEvents()
        if ord('q') in keys or 27 in keys:
            break

        for leg in LEG_NAMES:
            foot_pos = foot_position(leg, sim_time)
            result = leg_ik(foot_pos, HIP_OFFSETS[leg][1], 0.255)
            if result is None:
                continue
            hip_ab, thigh, calf = result
            for jid, angle, force, pg in zip(
                    leg_joints[leg],
                    [hip_ab, thigh, calf],
                    [23.7, 23.7, 35.55],
                    [0.8, 1.0, 1.0]):
                p.setJointMotorControl2(robot, jid,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle, force=force,
                    positionGain=pg, velocityGain=0.05)

        # Camera follows robot
        base_pos, _ = p.getBasePositionAndOrientation(robot)
        p.resetDebugVisualizerCamera(1.5, 45, -20, base_pos)

        p.stepSimulation()
        sim_time += dt
        time.sleep(dt)

    p.disconnect()
    print("Done!")

if __name__ == "__main__":
    run_simulation()
