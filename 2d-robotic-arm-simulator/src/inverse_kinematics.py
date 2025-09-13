# src/inverse_kinematics.py
import numpy as np

def inverse_kinematics_2link(x, y, L1, L2, eps=1e-9):
    """
    Solve IK for a planar 2-link manipulator.
    Returns None if unreachable, else returns two solutions:
      ((theta1_a, theta2_a), (theta1_b, theta2_b))
    Angles are in radians.
    """
    r2 = x*x + y*y
    r = np.sqrt(r2)
    # Reachability check (triangle inequality)
    if r > (L1 + L2) + eps or r < abs(L1 - L2) - eps:
        return None

    cos_theta2 = (r2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    # numerical safety
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2_a = np.arccos(cos_theta2)   # elbow-down
    theta2_b = -theta2_a               # elbow-up

    k1 = L1 + L2 * cos_theta2
    k2_a = L2 * np.sin(theta2_a)
    k2_b = L2 * np.sin(theta2_b)

    phi = np.arctan2(y, x)
    theta1_a = phi - np.arctan2(k2_a, k1)
    theta1_b = phi - np.arctan2(k2_b, k1)

    # normalize to [-pi, pi]
    def wrap(a):
        return np.arctan2(np.sin(a), np.cos(a))

    return (wrap(theta1_a), wrap(theta2_a)), (wrap(theta1_b), wrap(theta2_b))
