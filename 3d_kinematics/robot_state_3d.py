"""
Single source of truth for 6 DOF robot configuration.
All modules import from here.
"""

from enum import Enum

class Mode(Enum):
    MANUAL = "Manual"
    IK = "IK"

# UR5-like robot: 6 revolute joints
# Joint angles in degrees - UR5 home position
theta_deg = [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]  # Initial joint angles (UR5 home)
target_xyz = None              # IK target (x, y, z)
mode = Mode.MANUAL            # Current mode

def get_parameters():
    """Return copies to prevent mutation."""
    return theta_deg.copy()

def set_parameters(new_theta):
    global theta_deg
    theta_deg[:] = new_theta

def set_target(x, y, z):
    global target_xyz
    # Ensure all values are not None
    if x is None or y is None or z is None:
        return
    target_xyz = (float(x), float(y), float(z))

def get_target():
    return target_xyz

def set_mode(new_mode: Mode):
    global mode
    mode = new_mode

def get_mode():
    return mode

