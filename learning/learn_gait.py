import numpy as np

STEP_LENGTH = 0.08   # how far forward the foot moves (8cm)
STEP_HEIGHT = 0.055  # how high the foot lifts (5.5cm)
PERIOD      = 0.6    # seconds for one full gait cycle

def swing_trajectory(t):
    # t goes from 0 to 1 during swing phase
    angle = t * np.pi   # convert to 0 → π
    x = (STEP_LENGTH / 2) * np.cos(angle)
    z = STEP_HEIGHT * np.sin(angle)
    return x, z

def foot_position(phase):
    # phase goes from 0 to 1 (full gait cycle)
    
    if phase < 0.5:
        # SWING phase — foot in air
        t = phase / 0.5   # scale to 0→1
        x, z_lift = swing_trajectory(t)
        z = 0.27 - z_lift  # 0.27 = standing depth, subtract to lift up
    else:
        # STANCE phase — foot on ground
        t = (phase - 0.5) / 0.5   # scale to 0→1
        x = (STEP_LENGTH / 2) - (STEP_LENGTH * t)
        z = 0.27  # stay on ground
    
    return x, z

# phase offset for each leg (trot pattern)
PHASE_OFFSET = {
    "FR": 0.0,   # front right
    "RL": 0.0,   # rear left  (same as FR — diagonal pair 1)
    "FL": 0.5,   # front left
    "RR": 0.5,   # rear right (same as FL — diagonal pair 2)
}

def trot(leg, time):
    # calculate phase for this specific leg
    phase = (time / PERIOD + PHASE_OFFSET[leg]) % 1.0
    return foot_position(phase)

# print full trot cycle for all 4 legs
print(f"{'Time':<8} {'FR':>12} {'RL':>12} {'FL':>12} {'RR':>12}")
print("-" * 56)

for i in range(11):
    t = i * 0.06   # 0 to 0.6 in 11 steps
    fr = trot("FR", t)
    rl = trot("RL", t)
    fl = trot("FL", t)
    rr = trot("RR", t)
    print(f"t={t:.2f}   FR z={fr[1]:.3f}   RL z={rl[1]:.3f}   FL z={fl[1]:.3f}   RR z={rr[1]:.3f}")