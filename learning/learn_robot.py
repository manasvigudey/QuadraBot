import pybullet as p
import pybullet_data
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from config.robot_config import *

# Setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load ground and robot
ground = p.loadURDF("plane.urdf")
robot = p.loadURDF("go1_improved_nomesh.urdf",
                    basePosition=[0, 0, 0.4],
                    baseOrientation=p.getQuaternionFromEuler([3.14159, 0, 0]))

# Build a map of joint names to joint indices
joint_map = {}
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    name = info[1].decode('utf-8')
    joint_type = info[2]
    if joint_type == 0:  # only revolute (motor) joints
        joint_map[name] = i

print("Actuated joints found:")
for name, idx in joint_map.items():
    print(f"  {name} → index {idx}")

    # Organize by leg
legs = {
    "FR": [joint_map["FR_hip_joint"], joint_map["FR_thigh_joint"], joint_map["FR_calf_joint"]],
    "FL": [joint_map["FL_hip_joint"], joint_map["FL_thigh_joint"], joint_map["FL_calf_joint"]],
    "RR": [joint_map["RR_hip_joint"], joint_map["RR_thigh_joint"], joint_map["RR_calf_joint"]],
    "RL": [joint_map["RL_hip_joint"], joint_map["RL_thigh_joint"], joint_map["RL_calf_joint"]],
}

print("\nLegs organized:")
for leg, indices in legs.items():
    print(f"  {leg} → hip={indices[0]}, thigh={indices[1]}, calf={indices[2]}")

    # Standing pose — send to all 12 joints
STAND_ANGLES = [0.0, 0.8, -1.6]  # hip, thigh, calf

print("\nSending standing pose...")
for leg, indices in legs.items():
    for joint_idx, angle in zip(indices, STAND_ANGLES):
        p.setJointMotorControl2(
            robot, joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle,
            force=30,
            positionGain=0.5,
            velocityGain=0.05
        )

# Run simulation
for i in range(500):
    p.stepSimulation()
    time.sleep(1/100)

p.disconnect()
print("Done!")