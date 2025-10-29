from sympy import Matrix, sin, cos, symbols, Eq, solve
from math import degrees


def compute_analytical_ik(target_x, target_y):
    l1 = 3.0
    l2 = 2.5
    l3 = 1.5

    theta1, theta2, theta3 = symbols('theta1 theta2 theta3', real=True)

    H1 = Matrix([[cos(theta1), -sin(theta1), 0],
                [sin(theta1), cos(theta1), 0],
                [0, 0, 1]])
    
    H2 = Matrix([[cos(theta2), -sin(theta2), l1],
                [sin(theta2), cos(theta2), 0],
                [0, 0, 1]])
    
    H3 = Matrix([[cos(theta3), -sin(theta3), l2],
                [sin(theta3), cos(theta3), 0],
                [0, 0, 1]])
    
    H4 = Matrix([[1, 0, l3],
                [0, 1, 0],
                [0, 0, 1]])
    
    H = H1*H2*H3*H4
    
    p_e = Matrix([[0],
                [0],
                [1]])
    
    p_0 = H * p_e

    target_phi = 0.0

    eqs = [Eq(p_0[0], target_x), Eq(p_0[1], target_y), Eq(theta1 + theta2 + theta3, target_phi)]

    solutions = solve(eqs, [theta1, theta2, theta3], dict=True)

    theta1_val = degrees(float(solutions[0][theta1]))
    theta2_val = degrees(float(solutions[0][theta2]))
    theta3_val = degrees(float(solutions[0][theta3]))

    return [theta1_val, theta2_val, theta3_val]