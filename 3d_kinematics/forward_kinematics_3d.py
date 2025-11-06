"""
Forward Kinematics for 3D Robot Arm

This file implements forward kinematics using rotation matrices.
The visualization script will import and use the compute_fk function from here.
"""

import numpy as np

# UR5 link parameters - maintaining original working configuration
# Link lengths and offsets
LINK_LENGTHS = [
    0.089159,   # Link 1 (base to joint 2) - vertical
    0.2125,     # Link 2 (joint 2 to joint 3) - horizontal, reduced by half
    0.196125,   # Link 3 (joint 3 to joint 4) - horizontal, reduced by half
    0.10915,    # Link 4 (joint 4 to joint 5) - vertical
    0.09465,    # Link 5 (joint 5 to joint 6) - vertical
    0.0823      # Link 6 (joint 6 to end-effector) - vertical
]

def rotation_matrix_z(theta):
    """Rotation matrix about z-axis."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def rotation_matrix_y(theta):
    """Rotation matrix about y-axis."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c,  0, s],
        [0,  1, 0],
        [-s, 0, c]
    ])

def compute_fk(joint_angles_deg):
    """
    Compute forward kinematics using rotation matrices.
    Simple sequential approach like 2D script - maintains chain consistency.
    """
    # Convert to radians
    theta = np.radians(joint_angles_deg)
    
    # Initialize: base at origin
    positions = [np.array([0.0, 0.0, 0.0])]
    transforms = []
    
    # Base transform
    T0 = np.eye(4)
    transforms.append(T0.copy())
    
    # Current position and orientation
    pos = np.array([0.0, 0.0, 0.0])
    R = np.eye(3)  # Current rotation matrix
    
    # Joint 1: rotates about z-axis, then moves up by LINK_LENGTHS[0]
    R1 = rotation_matrix_z(theta[0])
    R = R @ R1
    # Move up in z direction (in world frame after rotation)
    pos = pos + R @ np.array([0.0, 0.0, LINK_LENGTHS[0]])
    positions.append(pos.copy())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    transforms.append(T.copy())
    
    # Joint 2: rotates about y-axis (pitch down), then moves forward/back
    R2 = rotation_matrix_y(theta[1])
    R = R @ R2
    # Move in x direction (forward/back after rotation)
    pos = pos + R @ np.array([LINK_LENGTHS[1], 0.0, 0.0])
    positions.append(pos.copy())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    transforms.append(T.copy())
    
    # Joint 3: rotates about y-axis (pitch), then moves forward/back
    R3 = rotation_matrix_y(theta[2])
    R = R @ R3
    pos = pos + R @ np.array([LINK_LENGTHS[2], 0.0, 0.0])
    positions.append(pos.copy())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    transforms.append(T.copy())
    
    # Joint 4: rotates about z-axis, then moves up
    R4 = rotation_matrix_z(theta[3])
    R = R @ R4
    pos = pos + R @ np.array([0.0, 0.0, LINK_LENGTHS[3]])
    positions.append(pos.copy())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    transforms.append(T.copy())
    
    # Joint 5: rotates about y-axis, then moves up
    R5 = rotation_matrix_y(theta[4])
    R = R @ R5
    pos = pos + R @ np.array([0.0, 0.0, LINK_LENGTHS[4]])
    positions.append(pos.copy())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    transforms.append(T.copy())
    
    # Joint 6: rotates about z-axis, then moves up
    R6 = rotation_matrix_z(theta[5])
    R = R @ R6
    pos = pos + R @ np.array([0.0, 0.0, LINK_LENGTHS[5]])
    positions.append(pos.copy())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    transforms.append(T.copy())
    
    # End-effector is at the last position
    ee_position = positions[-1]
    
    return ee_position, positions, transforms
