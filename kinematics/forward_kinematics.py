from sympy import Matrix, rad, sin, cos

def compute_fk(robot_parameters):
    l1 = robot_parameters[0][0]
    l2 = robot_parameters[0][1]
    l3 = robot_parameters[0][2]
    theta1 = rad(robot_parameters[1][0])
    theta2 = rad(robot_parameters[1][1])
    theta3 = rad(robot_parameters[1][2])

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

    x = float(H[0, 2])
    y = float(H[1, 2])

    return x, y

