import numpy as np
from forward_kinematics_3d import compute_fk

def compute_numerical_ik(target_x, target_y, target_z, initial_angles_deg=None, 
                         max_iterations=50, tolerance=5e-4, alpha=0.5):
    if initial_angles_deg is None:
        from robot_state_3d import get_parameters
        initial_angles_deg = get_parameters()
    
    current_angles = np.array(initial_angles_deg, dtype=float)
    target = np.array([target_x, target_y, target_z])
    
    adaptive_alpha = alpha
    
    for iteration in range(max_iterations):
        current_pos = np.array(compute_fk(current_angles.tolist()))
        
        error = target - current_pos
        error_norm = np.linalg.norm(error)
        
        if error_norm < tolerance:
            return current_angles.tolist()
        
        J = compute_jacobian(current_angles)
        
        lambda_damping = 0.01
        J_pseudo_inv = np.linalg.pinv(J)
        
        delta_theta = J_pseudo_inv @ error
        
        if error_norm > 0.1:
            step_size = adaptive_alpha * 1.5  
        else:
            step_size = adaptive_alpha  
        
        delta_theta *= step_size
        
        current_angles += delta_theta
        
    final_pos = np.array(compute_fk(current_angles.tolist()))
    final_error = np.linalg.norm(target - final_pos)
    if final_error < 0.01: 
        return current_angles.tolist()
    
    print(f"Numerical IK failed to converge after {max_iterations} iterations. Final error: {final_error:.6f}")
    return None


def compute_jacobian(joint_angles_deg):
    epsilon = 1e-6  
    
    current_pos = np.array(compute_fk(joint_angles_deg))
    J = np.zeros((3, 6))
    
    for i in range(6):
        perturbed_angles = joint_angles_deg.copy()
        perturbed_angles[i] += epsilon
        
        perturbed_pos = np.array(compute_fk(perturbed_angles))
        
        J[:, i] = (perturbed_pos - current_pos) / epsilon
    
    return J
