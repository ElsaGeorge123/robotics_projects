# src/forward_kinematics.py
import numpy as np

def forward_kinematics(link_lengths, thetas):
    """
    Compute sequential joint positions for an n-link planar arm.
    link_lengths: list or array [L1, L2, ...]
    thetas: list or array [theta1, theta2, ...] in radians (joint angles)
    Returns: list of (x,y) positions for base and each joint/end-effector.
    """
    positions = [(0.0, 0.0)]
    x, y = 0.0, 0.0
    angle = 0.0
    for L, th in zip(link_lengths, thetas):
        angle += th
        x += L * np.cos(angle)
        y += L * np.sin(angle)
        positions.append((x, y))
    return positions
