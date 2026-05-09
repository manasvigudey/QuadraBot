import numpy as np
L_THIGH = 0.213   # thigh length in meters
L_CALF  = 0.255   # improved calf length in meters
L_HIP   = 0.08    # hip lateral offset in meters

def leg_ik(px, pz):
    # px = how far forward the foot is
    # pz = how far down the foot is

    # straight line distance from hip to foot
    distance = np.sqrt(px**2 + pz**2)

    # cosine rule to find knee angle
    cos_knee = (L_THIGH**2 + L_CALF**2 - distance**2) / (2 * L_THIGH * L_CALF)
    cos_knee = np.clip(cos_knee, -1, 1)
    knee_angle = -np.arccos(cos_knee)

    # calculate thigh angle
    alpha = np.arctan2(px, pz)
    beta = np.arccos(np.clip((L_THIGH**2 + distance**2 - L_CALF**2) / (2 * L_THIGH * distance), -1, 1))
    thigh_angle = alpha - beta

    print("Distance from hip to foot:", distance)
    print("Knee angle in degrees:", np.degrees(knee_angle))
    print("Thigh angle in degrees:", np.degrees(thigh_angle))


leg_ik(0.0, 0.468)