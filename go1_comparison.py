"""
GO1 — Original vs Improved Comparison
Two robots walking side by side:
  Left  → Original Go1 parameters (calf 213mm)
  Right → Improved parameters (calf 255mm)
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

#parameters for both robots
# ── ORIGINAL Go1 parameters ──
ORIG = {
    "calf":        0.213,
    "step_length": 0.08,
    "step_height": 0.055,
    "period":      0.6,
}

# ── IMPROVED parameters ──
IMPR = {
    "calf":        0.255,
    "step_length": 0.06,
    "step_height": 0.045,
    "period":      0.7,
}

# ── Shared parameters ──
L_HIP    = 0.08
L_THIGH  = 0.213
STAND_HEIGHT = 0.27
LEG_NAMES = ["FR", "FL", "RR", "RL"]

HIP_OFFSETS = {
    "FR": ( 0.1881, -0.04675),
    "FL": ( 0.1881,  0.04675),
    "RR": (-0.1881, -0.04675),
    "RL": (-0.1881,  0.04675),
}

PHASE_OFFSET = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}

#3D IK function
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

#gait function
def foot_position(leg, t, params, x_dir=1):
    period      = params["period"]
    step_length = params["step_length"]
    step_height = params["step_height"]
    
    phase = (t / period + PHASE_OFFSET[leg]) % 1.0
    hip_y = HIP_OFFSETS[leg][1]
    
    if phase < 0.5:
        sw = phase / 0.5
        x = x_dir * (step_length / 2) * np.cos(np.pi * sw)
        z = STAND_HEIGHT - step_height * np.sin(np.pi * sw)
    else:
        st = (phase - 0.5) / 0.5
        x = x_dir * -(step_length / 2) * np.cos(np.pi * st)
        z = STAND_HEIGHT
    
    y = hip_y
    return [x, y, z]

#joint mapping function
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
        cameraDistance=2.5,
        cameraYaw=90,
        cameraPitch=-20,
        cameraTargetPosition=[0.3, 0, 0.1]
    )

    # Ground
    ground = p.loadURDF("plane.urdf")
    p.changeDynamics(ground, -1, lateralFriction=0.8)

    # Load TWO robots — side by side
    orn_orig = p.getQuaternionFromEuler([3.14159, 0, 0])
    orn_impr = p.getQuaternionFromEuler([3.14159, 0, 0])

    robot_orig = p.loadURDF("go1_original_nomesh.urdf",
                            [0, -0.8, 0.35], orn_orig,
                            flags=p.URDF_USE_INERTIA_FROM_FILE)

    robot_impr = p.loadURDF("go1_improved_nomesh.urdf",
                            [0,  0.4, 0.35], orn_impr,
                            flags=p.URDF_USE_INERTIA_FROM_FILE)

    # Labels
    p.addUserDebugText("ORIGINAL", [-0.5, -0.4, 0.45],
                       textColorRGB=[1, 0.2, 0.2], textSize=2.0)
    p.addUserDebugText("IMPROVED", [0,  0.8, 0.45],
                       textColorRGB=[0.2, 1, 0.2], textSize=2.0)

    # Joint maps
    joints_orig = get_leg_joints(robot_orig)
    joints_impr = get_leg_joints(robot_impr)

    # settle and main loop
# Settle both robots
    print("Settling...")
    for _ in range(500):
        for robot, joints in [(robot_orig, joints_orig), (robot_impr, joints_impr)]:
            for leg, ids in joints.items():
                for jid, angle in zip(ids, [0.0, 0.8, -1.6]):
                    p.setJointMotorControl2(robot, jid,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle, force=35,
                        positionGain=0.5, velocityGain=0.05)
        p.stepSimulation()

    print("Comparing...")
    sim_time = 0.0
    dt = 1.0 / 500.0

    while p.isConnected():
        keys = p.getKeyboardEvents()
        if ord('q') in keys or 27 in keys:
            break

        for leg in LEG_NAMES:
            # Original robot
            foot_o = foot_position(leg, sim_time, ORIG, x_dir=1)
            result_o = leg_ik(foot_o, HIP_OFFSETS[leg][1], ORIG["calf"])
            if result_o:
                for jid, angle, force, pg in zip(
                        joints_orig[leg], result_o,
                        [23.7, 23.7, 35.55], [0.8, 1.0, 1.0]):
                    p.setJointMotorControl2(robot_orig, jid,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle, force=force,
                        positionGain=pg, velocityGain=0.05)

            # Improved robot
            foot_i = foot_position(leg, sim_time, IMPR)
            result_i = leg_ik(foot_i, HIP_OFFSETS[leg][1], IMPR["calf"])
            if result_i:
                for jid, angle, force, pg in zip(
                        joints_impr[leg], result_i,
                        [23.7, 23.7, 35.55], [0.8, 1.0, 1.0]):
                    p.setJointMotorControl2(robot_impr, jid,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle, force=force,
                        positionGain=pg, velocityGain=0.05)

        # Camera follows midpoint between both robots
        pos_o, _ = p.getBasePositionAndOrientation(robot_orig)
        pos_i, _ = p.getBasePositionAndOrientation(robot_impr)
        mid = [(pos_o[0]+pos_i[0])/2, (pos_o[1]+pos_i[1])/2, 0.1]
        p.resetDebugVisualizerCamera(2.5, 90, -20, mid)

        p.stepSimulation()
        sim_time += dt
        time.sleep(dt)

    p.disconnect()
    print("Done!")

if __name__ == "__main__":
    run_simulation()
