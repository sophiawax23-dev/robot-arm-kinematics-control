import numpy as np

# Lengths of the two robot arm segments (meters)
L1 = 1.0
L2 = 0.8

def forward_kinematics(theta1, theta2):
    """Calculate the end-effector position of a 2-link planar arm."""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

# Example: joint angles in radians
theta1 = np.radians(30)
theta2 = np.radians(45)

x, y = forward_kinematics(theta1, theta2)
print(f"End-effector position: x={x:.3f}, y={y:.3f}")
  
