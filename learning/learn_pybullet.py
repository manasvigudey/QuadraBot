import pybullet as p
import pybullet_data
import time

# Connect and setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load ground
ground = p.loadURDF("plane.urdf")
print("Ground ID:", ground)

# Load our robot
robot = p.loadURDF("go1_improved_nomesh.urdf",
                    basePosition=[0, 0, 0.4],
                    baseOrientation=p.getQuaternionFromEuler([3.14159, 0, 0]))

print("Robot ID:", robot)
print("Number of joints:", p.getNumJoints(robot))

# Print all joint names
print("\nJoint List:")
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    print(f"  Joint {i}: {info[1].decode('utf-8')} — type: {info[2]}")

# Move FR_thigh_joint (joint index 3) to 45 degrees
p.setJointMotorControl2(
    robot, 3,
    controlMode=p.POSITION_CONTROL,
    targetPosition=0.785,  # 45 degrees in radians
    force=20
)

# Keep window open for 5 seconds
for i in range(500):
    p.stepSimulation()
    time.sleep(1/100)

p.disconnect()