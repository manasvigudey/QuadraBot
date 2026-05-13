import numpy as np
import matplotlib.pyplot as plt
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from config.robot_config import *

# ── Workspace — sweep all joint angles ──
def compute_workspace(l_thigh, l_calf):
    thigh_angles = np.linspace(-0.686, 4.501, 200)  # thigh joint limits
    calf_angles  = np.linspace(-2.818, -0.888, 200)  # calf joint limits
    
    x_points = []
    z_points = []
    
    for t2 in thigh_angles:
        for t3 in calf_angles:
            x = l_thigh * np.sin(t2) + l_calf * np.sin(t2 + t3)
            z = l_thigh * np.cos(t2) + l_calf * np.cos(t2 + t3)
            x_points.append(x)
            z_points.append(z)
    
    return x_points, z_points

# ── Foot trajectory — half ellipse arc ──
def compute_trajectory():
    t_values = np.linspace(0, 1, 100)
    x_traj = []
    z_traj = []
    
    for t in t_values:
        angle = t * np.pi
        x = (STEP_LENGTH / 2) * np.cos(angle)
        z = STEP_HEIGHT * np.sin(angle)
        x_traj.append(x)
        z_traj.append(z)
    
    return x_traj, z_traj

# ── Plot ──
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
fig.suptitle("GO1 Kinematic Analysis", fontsize=14, fontweight='bold')

# LEFT PLOT — Workspace comparison
x_orig, z_orig = compute_workspace(0.213, 0.213)  # original
x_impr, z_impr = compute_workspace(0.213, 0.255)  # improved

ax1.scatter(x_orig, z_orig, s=0.1, color='grey', alpha=0.3, label='Original (calf 213mm)')
ax1.scatter(x_impr, z_impr, s=0.1, color='blue', alpha=0.3, label='Improved (calf 255mm)')
ax1.set_title("Foot Workspace Comparison")
ax1.set_xlabel("Forward/Backward (m)")
ax1.set_ylabel("Up/Down (m)")
ax1.legend()
ax1.grid(True)
ax1.invert_yaxis()

# RIGHT PLOT — Foot trajectory comparison
x_traj, z_traj = compute_trajectory()

ground_orig = 0.213 + 0.213 * 0.85
ground_impr = 0.213 + 0.255 * 0.85

ax2.plot(x_traj, [ground_orig - z for z in z_traj],
         color='grey', linewidth=2, label='Original (calf 213mm)')
ax2.plot(x_traj, [ground_impr - z for z in z_traj],
         color='blue', linewidth=2, label='Improved (calf 255mm)')
ax2.axhline(y=ground_orig, color='grey', linestyle='--', alpha=0.5)
ax2.axhline(y=ground_impr, color='blue', linestyle='--', alpha=0.5)
ax2.set_title("Foot Swing Trajectory — Improved Leg Reaches Further Down")
ax2.set_xlabel("Forward/Backward (m)")
ax2.set_ylabel("Depth from Hip (m)")
ax2.legend()
ax2.grid(True)
ax2.invert_yaxis()

ax2.annotate('Improved reaches\n3.6cm deeper', xy=(0, 0.430), 
             xytext=(0.02, 0.41), color='blue', fontsize=9,
             arrowprops=dict(arrowstyle='->', color='blue'))

plt.savefig('results/workspace_comparison.png', dpi=150, bbox_inches='tight')
print("Saved to results/workspace_comparison.png")

plt.show()