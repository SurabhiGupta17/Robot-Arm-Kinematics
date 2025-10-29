from math import atan2, cos, sin, sqrt, degrees, acos
import numpy as np

def compute_analytical_ik(target_x, target_y):
    l1 = 4.0
    l2 = 3.5
    l3 = 2.5
    
    desired_ee_angle = atan2(target_y, target_x)
    
    wx = target_x - l3 * cos(desired_ee_angle)
    wy = target_y - l3 * sin(desired_ee_angle)

    dw = sqrt(wx**2 + wy**2)
    
    if dw > (l1 + l2) or dw < abs(l1 - l2):
        print(f"Unreachable! wrist distance={dw:.2f}, range=[{abs(l1-l2):.2f}, {l1+l2:.2f}]")
        return None
    
    cos_theta2 = (dw**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = acos(cos_theta2)
    
    alpha = atan2(wy, wx)
    beta = atan2(l2 * sin(theta2), l1 + l2 * cos(theta2))
    theta1 = alpha - beta
    
    theta3 = desired_ee_angle - theta1 - theta2
    
    return [degrees(theta1), degrees(theta2), degrees(theta3)]