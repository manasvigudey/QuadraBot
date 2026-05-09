"""
Go1 Improved — PyBullet Trot Simulation
========================================
Loads go1_improved_nomesh.urdf and runs a trot gait using analytical IK.

Requirements:
    pip install pybullet numpy

Usage:
    python go1_simulation.py

Controls (keyboard in PyBullet window):
    Q / ESC  — quit
    S        — screenshot
    Space    — pause/resume
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import sys

# ─────────────────────────────────────────────
# ROBOT PARAMETERS  (must match your URDF)
# ─────────────────────────────────────────────

# Improved link lengths
L_HIP   = 0.08    # lateral hip offset (m)
L_THIGH = 0.213   # thigh length (m)
L_CALF  = 0.255   # IMPROVED calf length (m)

# Standing height from trunk centre to ground
STAND_HEIGHT = 0.27   # (m) — tune this if robot floats or clips ground

# Joint indices in URDF (order: hip, thigh, calf per leg)
# Go1 leg order: FR, FL, RR, RL
LEG_NAMES = ["FR", "FL", "RR", "RL"]

# Hip positions on trunk (x, y) relative to trunk centre
HIP_OFFSETS = {
    "FR": ( 0.1881, -0.04675),
    "FL": ( 0.1881,  0.04675),
    "RR": (-0.1881, -0.04675),
    "RL": (-0.1881,  0.04675),
}

# Trot diagonal pairs (move together)
TROT_PAIRS = [["FR", "RL"], ["FL", "RR"]]

# ─────────────────────────────────────────────
# INVERSE KINEMATICS  (analytical, 3-DOF leg)
# ─────────────────────────────────────────────

def leg_ik(foot_pos_body, hip_offset_y):
    """
    Compute joint angles (hip_ab, thigh, calf) given desired foot
    position relative to trunk centre.

    foot_pos_body : [x, y, z] desired foot in body frame (z down = positive)
    hip_offset_y  : lateral offset of hip from trunk centre (+ = left, - = right)

    Returns (hip_ab, thigh, calf) in radians, or None if unreachable.
    """
    px, py, pz = foot_pos_body

    # ── Hip abduction (roll) ──────────────────
    # Vector from hip joint to foot in the lateral-vertical plane
    dy = py - hip_offset_y
    dz = pz
    hip_ab = np.arctan2(dy, dz)

    # ── Project into sagittal plane ───────────
    # Distance from hip joint to foot after removing lateral component
    lat_dist = np.sqrt(dy**2 + dz**2) - L_HIP
    l = np.sqrt(px**2 + lat_dist**2)   # 2D distance to foot

    if l > (L_THIGH + L_CALF) or l < abs(L_THIGH - L_CALF):
        return None  # unreachable

    # ── Calf angle (cosine rule) ──────────────
    cos_calf = (L_THIGH**2 + L_CALF**2 - l**2) / (2 * L_THIGH * L_CALF)
    cos_calf = np.clip(cos_calf, -1, 1)
    calf = -(np.pi - np.arccos(cos_calf))   # negative = knee bent backward

    # ── Thigh angle ───────────────────────────
    alpha = np.arctan2(px, lat_dist)
    beta  = np.arccos(np.clip((L_THIGH**2 + l**2 - L_CALF**2) /
                               (2 * L_THIGH * l), -1, 1))
    thigh = alpha - beta

    return hip_ab, thigh, calf

# ─────────────────────────────────────────────
# GAIT PLANNER  — trot with half-ellipse swing
# ─────────────────────────────────────────────

class TrotGait:
    def __init__(self, step_length=0.08, step_height=0.055,
                 period=0.6, body_height=STAND_HEIGHT):
        self.step_length = step_length
        self.step_height = step_height
        self.period      = period        # seconds per full cycle
        self.body_height = body_height

        # Phase offset per leg (trot: diagonal pairs in sync, 0.5 offset between pairs)
        self.phase_offset = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}

    def foot_position(self, leg, t):
        """
        Returns desired foot position [x, y, z] in body frame at time t.
        z is positive downward (depth from hip).
        """
        phase = (t / self.period + self.phase_offset[leg]) % 1.0
        hip_y = HIP_OFFSETS[leg][1]

        if phase < 0.5:
            # ── SWING phase ───────────────────
            sw = phase / 0.5   # 0→1 over swing
            x = (self.step_length / 2) * np.cos(np.pi * sw)
            z = self.body_height - self.step_height * np.sin(np.pi * sw)
        else:
            # ── STANCE phase ──────────────────
            st = (phase - 0.5) / 0.5   # 0→1 over stance
            x = -(self.step_length / 2) * np.cos(np.pi * st)
            z = self.body_height

        y = hip_y   # foot stays under hip laterally
        return [x, y, z]

# ─────────────────────────────────────────────
# PYBULLET SETUP
# ─────────────────────────────────────────────

def find_urdf():
    """Search common locations for the improved URDF."""
    candidates = [
        "go1_improved_nomesh.urdf",
        os.path.join(os.path.dirname(__file__), "go1_improved_nomesh.urdf"),
        os.path.expanduser("~/go1_improved_nomesh.urdf"),
        os.path.expanduser("~/Downloads/go1_improved_nomesh.urdf"),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return None

def get_joint_map(robot_id):
    """
    Build a dict mapping 'LEG_jointtype' → joint index.
    e.g. 'FR_hip_joint' → 0
    """
    joint_map = {}
    n = p.getNumJoints(robot_id)
    for i in range(n):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode("utf-8")
        joint_map[name] = i
    return joint_map

def get_leg_joint_ids(joint_map):
    """Returns dict: leg → [hip_id, thigh_id, calf_id]"""
    leg_joints = {}
    for leg in LEG_NAMES:
        hip   = joint_map.get(f"{leg}_hip_joint")
        thigh = joint_map.get(f"{leg}_thigh_joint")
        calf  = joint_map.get(f"{leg}_calf_joint")
        if None in (hip, thigh, calf):
            print(f"⚠️  Could not find joints for leg {leg}")
            print(f"   Available joints: {list(joint_map.keys())[:10]} ...")
        else:
            leg_joints[leg] = [hip, thigh, calf]
    return leg_joints

# ─────────────────────────────────────────────
# MAIN SIMULATION
# ─────────────────────────────────────────────

def run_simulation():
    # ── Connect ──────────────────────────────
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # ── Camera ───────────────────────────────
    p.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[0, 0, 0]
    )
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    # ── Ground ───────────────────────────────
    ground = p.loadURDF("plane.urdf")
    p.changeDynamics(ground, -1, lateralFriction=0.8)

    # ── Load Robot ───────────────────────────
    urdf_path = find_urdf()
    if urdf_path is None:
        print("\n❌ go1_improved_nomesh.urdf not found!")
        print("   Place go1_improved_nomesh.urdf in the same folder as this script.")
        print("   Or update the 'candidates' list in find_urdf().\n")
        p.disconnect()
        sys.exit(1)

    print(f"✅ Loading URDF: {urdf_path}")
    start_pos = [0, 0, 0.35]   # spawn slightly above ground, will settle
    start_orn = p.getQuaternionFromEuler([3.14159, 0, 0])

    robot = p.loadURDF(
        urdf_path,
        start_pos,
        start_orn,
        flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE,
        useFixedBase=False
    )

    # ── Joint Map ────────────────────────────
    joint_map  = get_joint_map(robot)
    leg_joints = get_leg_joint_ids(joint_map)

    if len(leg_joints) < 4:
        print("⚠️  Not all legs found. Check URDF joint names.")

    # Enable all actuated joints
    for leg, ids in leg_joints.items():
        for jid in ids:
            p.enableJointForceTorqueSensor(robot, jid, True)

    # ── Stand Still for 1 second (settle) ────
    print("⏳ Settling robot...")
    gait = TrotGait(step_length=0.08, step_height=0.055, period=0.6)

    STAND_ANGLES = {"FR": [0.0,  0.8, -1.6],
                    "FL": [0.0,  0.8, -1.6],
                    "RR": [0.0,  0.8, -1.6],
                    "RL": [0.0,  0.8, -1.6]}

    for _ in range(500):
        for leg, ids in leg_joints.items():
            for jid, angle in zip(ids, STAND_ANGLES[leg]):
                p.setJointMotorControl2(
                    robot, jid,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=35,
                    positionGain=0.5,
                    velocityGain=0.05
                )
        p.stepSimulation()

    print("🚶 Starting trot gait...")

    # ── Simulation Loop ───────────────────────
    sim_time   = 0.0
    dt         = 1.0 / 500.0   # 500 Hz simulation
    paused     = False
    screenshot = False

    try:
        while p.isConnected():
            # Keyboard events
            keys = p.getKeyboardEvents()
            if ord('q') in keys or 27 in keys:   # Q or ESC
                break
            if ord(' ') in keys and keys[ord(' ')] == p.KEY_WAS_TRIGGERED:
                paused = not paused
                print("⏸  Paused" if paused else "▶️  Resumed")
            if ord('s') in keys and keys[ord('s')] == p.KEY_WAS_TRIGGERED:
                p.getCameraImage(1280, 720,
                                 renderer=p.ER_BULLET_HARDWARE_OPENGL)
                print("📸 Screenshot taken")

            if paused:
                time.sleep(0.01)
                continue

            # ── IK + Joint Control ────────────
            for leg, ids in leg_joints.items():
                foot_pos = gait.foot_position(leg, sim_time)
                result   = leg_ik(foot_pos, HIP_OFFSETS[leg][1])

                if result is None:
                    continue  # skip if unreachable

                hip_ab, thigh, calf = result
                target_angles = [hip_ab, thigh, calf]
                forces         = [23.7,   23.7,  35.55]
                p_gains        = [0.8,    1.0,   1.0]
                d_gains        = [0.05,   0.05,  0.05]

                for jid, angle, force, pg, dg in zip(
                        ids, target_angles, forces, p_gains, d_gains):
                    p.setJointMotorControl2(
                        robot, jid,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle,
                        force=force,
                        positionGain=pg,
                        velocityGain=dg
                    )

            # ── Camera Follow ─────────────────
            base_pos, _ = p.getBasePositionAndOrientation(robot)
            p.resetDebugVisualizerCamera(
                cameraDistance=1.2,
                cameraYaw=45 + sim_time * 5,   # slowly orbit
                cameraPitch=-25,
                cameraTargetPosition=base_pos
            )

            p.stepSimulation()
            sim_time += dt
            time.sleep(dt)   # real-time playback

    except KeyboardInterrupt:
        pass

    print("👋 Simulation ended.")
    p.disconnect()

# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════╗
║   Go1 Improved — PyBullet Trot Simulation    ║
║                                              ║
║   Controls:                                  ║
║     SPACE  — pause / resume                  ║
║     S      — screenshot                      ║
║     Q/ESC  — quit                            ║
╚══════════════════════════════════════════════╝
    """)
    run_simulation()
