import numpy as np
from sympy import Matrix, rad, cos, sin

LINK_LENGTHS = [
    0.089159, 
    0.2125,  
    0.196125,  
    0.10915,
    0.09465 
]

def compute_fk(joint_angles_deg):
    theta1 = rad(joint_angles_deg[0])
    theta2 = rad(joint_angles_deg[1])
    theta3 = rad(joint_angles_deg[2])
    theta4 = rad(joint_angles_deg[3])
    theta5 = rad(joint_angles_deg[4])
    theta6 = rad(joint_angles_deg[5])

    l1 = LINK_LENGTHS[0]
    l2 = LINK_LENGTHS[1]
    l3 = LINK_LENGTHS[2]
    l4 = LINK_LENGTHS[3]
    l5 = LINK_LENGTHS[4]

    H1 = Matrix([
        [cos(theta1), -sin(theta1), 0, 0],
        [sin(theta1), cos(theta1), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H2 = Matrix([
        [cos(theta2), 0, sin(theta2), 0],
        [0, 1, 0, 0],
        [-sin(theta2), 0, cos(theta2), l1],
        [0, 0, 0, 1]
    ])

    H3 = Matrix([
        [cos(theta3), 0, sin(theta3), l2],
        [0, 1, 0, 0],
        [-sin(theta3), 0, cos(theta3), 0],
        [0, 0, 0, 1]
    ])

    H4 = Matrix([
        [cos(theta4), -sin(theta4), 0, l3],
        [sin(theta4), cos(theta4), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    H5 = Matrix([
        [cos(theta5), 0, sin(theta5), 0],
        [0, 1, 0, 0],
        [-sin(theta5), 0, cos(theta5), 0],
        [0, 0, 0, 1]
    ])

    H6 = Matrix([
        [cos(theta6), -sin(theta6), 0, 0],
        [sin(theta6), cos(theta6), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    H_ee = Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, l4 + l5],
        [0, 0, 0, 1]
    ])

    H = H1*H2*H3*H4*H5*H6*H_ee

    x = float(H[0, 3])
    y = float(H[1, 3])
    z = float(H[2, 3])

    return x, y, z
