"""
Single source of truth for 6 DOF robot configuration.
All modules import from here.
"""

from enum import Enum

class Mode(Enum):
    MANUAL = "Manual"
    IK = "IK"

theta_deg = [-39.0, -84.0, -44.0, -34.0, -29.0, 0.0]  
target_xyz = None            
mode = Mode.MANUAL            

def get_parameters():
    """Return copies to prevent mutation."""
    return theta_deg.copy()

def set_parameters(new_theta):
    global theta_deg
    theta_deg[:] = new_theta

def set_target(x, y, z):
    global target_xyz
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

