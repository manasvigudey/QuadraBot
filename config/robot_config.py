# robot_config.py
# All robot parameters in a single file

# ── Link lengths (meters) ──
L_HIP   = 0.08    # hip lateral offset
L_THIGH = 0.213   # thigh length
L_CALF  = 0.255   # improved calf length (original: 0.213)

# ── Gait parameters ──
STEP_LENGTH = 0.08    # how far forward foot moves (m)
STEP_HEIGHT = 0.055   # how high foot lifts (m)
PERIOD      = 0.6     # seconds per full gait cycle
STAND_DEPTH = 0.27    # how far down foot stands (m)

# ── Trot phase offsets ──
PHASE_OFFSET = {
    "FR": 0.0,
    "RL": 0.0,
    "FL": 0.5,
    "RR": 0.5,
}