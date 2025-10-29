"""
Single source of truth for robot configuration.
All modules import from here.
"""

from enum import Enum

class Mode(Enum):
    MANUAL = "Manual"
    IK = "IK"

L = [3.0, 2.5, 1.5]           # Link lengths
theta_deg = [30.0, 45.0, 30.0] # Joint angles (degrees)
target_xy = None              # IK target (x, y)
mode = Mode.MANUAL            # Current mode

def get_parameters():
    """Return copies to prevent mutation."""
    return [L.copy(), theta_deg.copy()]

def set_parameters(new_L, new_theta):
    global L, theta_deg
    L[:] = new_L
    theta_deg[:] = new_theta

def set_target(x, y):
    global target_xy
    target_xy = (x, y)

def get_target():
    return target_xy

def set_mode(new_mode: Mode):
    global mode
    mode = new_mode

def get_mode():
    return mode