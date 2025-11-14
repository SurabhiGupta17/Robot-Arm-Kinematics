import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, RadioButtons, Button
from matplotlib.animation import FuncAnimation

from robot_state_3d import (
    theta_deg, target_xyz, Mode,
    get_parameters, set_parameters, set_target, get_target, set_mode, get_mode
)

# Forward kinematics - import from user's implementation
# This maintains chain consistency like the 2D script
from forward_kinematics_3d import LINK_LENGTHS

INITIAL_THETA = [-39.0, -84.0, -44.0, -34.0, -29.0, 0.0]

# Rotation matrix helper functions for visualization
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

def compute_fk_full(joint_angles_deg):
    """
    Compute forward kinematics with full details for visualization.
    Returns end-effector position, all joint positions, and transforms.
    This uses the same FK logic as the user's compute_fk function.
    
    Args:
        joint_angles_deg: List of 6 joint angles in degrees
    
    Returns:
        tuple: (ee_position, positions, transforms)
            - ee_position: End-effector position (x, y, z)
            - positions: List of all joint positions
            - transforms: List of all transformation matrices
    """
    from sympy import Matrix, rad, cos, sin
    
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

    # Build transformation matrices (same as user's compute_fk)
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
        [-sin(theta5), 0, cos(theta5), l4],
        [0, 0, 0, 1]
    ])

    H6 = Matrix([
        [cos(theta6), -sin(theta6), 0, 0],
        [sin(theta6), cos(theta6), 0, 0],
        [0, 0, 1, l5],
        [0, 0, 0, 1]
    ])

    # Compute intermediate transforms to get joint positions
    # Each transform represents the cumulative transformation from base to that joint
    T1 = H1
    T2 = H1 * H2
    T3 = H1 * H2 * H3
    T4 = H1 * H2 * H3 * H4
    T5 = H1 * H2 * H3 * H4 * H5
    T6 = H1 * H2 * H3 * H4 * H5 * H6  # End-effector
    
    # Extract positions from transformation matrices
    positions = []
    transforms = []
    
    # Base position and transform
    base_pos = np.array([0.0, 0.0, 0.0])
    positions.append(base_pos)
    transforms.append(np.eye(4))
    
    # Convert sympy matrices to numpy and extract positions for each joint
    # Structure: base, joint1, joint2, joint3, joint4, joint5, joint6 (end-effector) = 7 positions
    # Transforms: base, T1, T2, T3, T4, T5, T6 = 7 transforms (one per position)
    transforms_list = [T1, T2, T3, T4, T5, T6]
    for T in transforms_list:
        # Convert sympy Matrix to numpy array
        T_np = np.array(T.tolist()).astype(float)
        transforms.append(T_np)
        
        # Extract position (translation part) - this is the joint position in world frame
        pos = np.array([float(T_np[0, 3]), float(T_np[1, 3]), float(T_np[2, 3])])
        positions.append(pos)
    
    # End-effector is at joint 6 position (last position)
    ee_position = positions[-1]
    
    return ee_position, positions, transforms

ee_text = None  
target_marker = None
target_label = None
sliders = [] 

animation_active = False
target_angles = None
current_angles = None
animation_progress = 0
animation_speed = 0.08

def smooth_lerp(current, target, factor=0.3):
    return current + (target - current) * factor

def safe_remove(artist):
    if artist is not None and hasattr(artist, 'remove') and artist.axes is not None:
        try:
            artist.remove()
        except (ValueError, AttributeError):
            pass

def draw_cylinder(ax, p1, p2, radius=0.02, color='#1f77b4', alpha=0.6, resolution=20):
    """
    Draw a cylinder between two points.
    
    Args:
        ax: 3D axes
        p1: Start point (x, y, z)
        p2: End point (x, y, z)
        radius: Cylinder radius
        color: Cylinder color
        alpha: Transparency (0-1)
        resolution: Number of points around the cylinder
    
    Returns:
        List of surface artists
    """
    # Vector from p1 to p2
    v = np.array(p2) - np.array(p1)
    length = np.linalg.norm(v)
    
    if length < 1e-6:
        return []
    
    # Normalize direction vector
    v = v / length
    
    # Create a circle in the XY plane
    theta = np.linspace(0, 2 * np.pi, resolution)
    x_circle = radius * np.cos(theta)
    y_circle = radius * np.sin(theta)
    z_circle = np.zeros_like(theta)
    
    # Find rotation axis and angle to align with v
    # Default direction is along z-axis
    z_axis = np.array([0, 0, 1])
    
    # Rotation axis (cross product)
    if np.abs(np.dot(v, z_axis)) > 0.99:  # Already aligned
        R = np.eye(3)
    else:
        rot_axis = np.cross(z_axis, v)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_angle = np.arccos(np.clip(np.dot(z_axis, v), -1, 1))
        
        # Rotation matrix (Rodrigues' formula)
        K = np.array([[0, -rot_axis[2], rot_axis[1]],
                      [rot_axis[2], 0, -rot_axis[0]],
                      [-rot_axis[1], rot_axis[0], 0]])
        R = np.eye(3) + np.sin(rot_angle) * K + (1 - np.cos(rot_angle)) * np.dot(K, K)
    
    # Rotate circle points
    circle_points = np.array([x_circle, y_circle, z_circle])
    rotated_points = R @ circle_points
    
    # Create cylinder surface along the length
    num_segments = max(2, int(length / radius * 2))  # Adaptive segmentation
    z_vals = np.linspace(0, length, num_segments)
    
    X = np.zeros((num_segments, resolution))
    Y = np.zeros((num_segments, resolution))
    Z = np.zeros((num_segments, resolution))
    
    for i, z in enumerate(z_vals):
        offset = z * v
        X[i, :] = p1[0] + rotated_points[0, :] + offset[0]
        Y[i, :] = p1[1] + rotated_points[1, :] + offset[1]
        Z[i, :] = p1[2] + rotated_points[2, :] + offset[2]
    
    # Draw the cylinder surface
    surfaces = []
    surf = ax.plot_surface(X, Y, Z, color=color, alpha=alpha, shade=True, 
                          linewidth=0, antialiased=True, zorder=1)
    surfaces.append(surf)
    
    # Draw end caps
    if length > radius * 2:
        # Start cap
        X_cap = np.tile(p1[0] + rotated_points[0, :], (1, 1))
        Y_cap = np.tile(p1[1] + rotated_points[1, :], (1, 1))
        Z_cap = np.tile(p1[2] + rotated_points[2, :], (1, 1))
        cap1 = ax.plot_surface(X_cap, Y_cap, Z_cap, color=color, alpha=alpha, zorder=1)
        surfaces.append(cap1)
        
        # End cap
        X_cap = np.tile(p2[0] + rotated_points[0, :], (1, 1))
        Y_cap = np.tile(p2[1] + rotated_points[1, :], (1, 1))
        Z_cap = np.tile(p2[2] + rotated_points[2, :], (1, 1))
        cap2 = ax.plot_surface(X_cap, Y_cap, Z_cap, color=color, alpha=alpha, zorder=1)
        surfaces.append(cap2)
    
    return surfaces

def draw_axes(ax, position, transform, axis_length=0.05, linewidth=2):
    """
    Draw RGB coordinate axes (X=red, Y=green, Z=blue) at a position.
    
    The axes are drawn to match matplotlib's coordinate system:
    - X axis (red) points along matplotlib's x-axis
    - Y axis (green) points along matplotlib's y-axis  
    - Z axis (blue) points along matplotlib's z-axis
    
    Args:
        ax: 3D axes
        position: Position (x, y, z) in matplotlib's coordinate system
        transform: 4x4 transformation matrix
        axis_length: Length of each axis
        linewidth: Line width
    
    Returns:
        List of line artists
    """
    # Extract rotation matrix from transform
    R = transform[:3, :3]
    
    # Unit vectors along the frame's local axes (in the frame's coordinate system)
    # These will be transformed to world coordinates by the rotation matrix
    x_local = np.array([axis_length, 0, 0])  # Local x-axis
    y_local = np.array([0, axis_length, 0])   # Local y-axis
    z_local = np.array([0, 0, axis_length])   # Local z-axis
    
    # Transform to world coordinates (matplotlib's coordinate system)
    x_axis = R @ x_local
    y_axis = R @ y_local
    z_axis = R @ z_local
    
    pos = np.array(position)
    
    # Draw X axis (red) - this should align with matplotlib's x-axis at the base
    line_x, = ax.plot([pos[0], pos[0] + x_axis[0]], 
                      [pos[1], pos[1] + x_axis[1]], 
                      [pos[2], pos[2] + x_axis[2]], 
                      'r-', linewidth=linewidth, zorder=5)
    
    # Draw Y axis (green) - this should align with matplotlib's y-axis at the base
    line_y, = ax.plot([pos[0], pos[0] + y_axis[0]], 
                      [pos[1], pos[1] + y_axis[1]], 
                      [pos[2], pos[2] + y_axis[2]], 
                      'g-', linewidth=linewidth, zorder=5)
    
    # Draw Z axis (blue) - this should align with matplotlib's z-axis at the base
    line_z, = ax.plot([pos[0], pos[0] + z_axis[0]], 
                      [pos[1], pos[1] + z_axis[1]], 
                      [pos[2], pos[2] + z_axis[2]], 
                      'b-', linewidth=linewidth, zorder=5)
    
    return [line_x, line_y, line_z]

def draw_gripper(ax, ee_position, transform, finger_length=0.08, finger_width=0.015, gap=0.03):
    """
    Draw a 2-finger gripper at the end-effector.
    
    Args:
        ax: 3D axes
        ee_position: End-effector position
        transform: End-effector transformation matrix
        finger_length: Length of each finger
        finger_width: Width of each finger
        gap: Gap between fingers
    
    Returns:
        List of surface artists
    """
    surfaces = []
    
    # Extract rotation matrix
    R = transform[:3, :3]
    
    # Gripper points along z-axis (pointing down)
    z_dir = R @ np.array([0, 0, -1])  # Pointing down
    x_dir = R @ np.array([1, 0, 0])   # Side direction
    
    # Base of gripper (small cylinder)
    base_radius = 0.02
    base_length = 0.03
    base_end = ee_position + z_dir * base_length
    
    base_cyl = draw_cylinder(ax, ee_position, base_end, radius=base_radius, 
                             color='#404040', alpha=0.8)
    if base_cyl:
        surfaces.extend(base_cyl)
    
    # Two fingers
    finger_start = base_end
    finger_end = finger_start + z_dir * finger_length
    
    # Left finger (offset by -gap/2)
    left_start = finger_start - x_dir * (gap / 2)
    left_end = finger_end - x_dir * (gap / 2)
    left_finger = draw_cylinder(ax, left_start, left_end, radius=finger_width, 
                                color='#606060', alpha=0.9)
    if left_finger:
        surfaces.extend(left_finger)
    
    # Right finger (offset by +gap/2)
    right_start = finger_start + x_dir * (gap / 2)
    right_end = finger_end + x_dir * (gap / 2)
    right_finger = draw_cylinder(ax, right_start, right_end, radius=finger_width, 
                                 color='#606060', alpha=0.9)
    if right_finger:
        surfaces.extend(right_finger)
    
    return surfaces

def draw_angle_arc_3d(ax, joint_pos, prev_transform, current_transform, joint_axis, 
                      angle_deg, radius=0.08, color='#ff6b6b'):
    """
    Draw angle arc in 3D space showing joint rotation.
    
    Args:
        ax: 3D axes
        joint_pos: Joint position
        prev_transform: Previous frame transform
        current_transform: Current frame transform
        joint_axis: Rotation axis (normalized vector)
        angle_deg: Joint angle in degrees
        radius: Arc radius
        color: Arc color
    
    Returns:
        List of artists (arc line and text)
    """
    from math import radians, cos, sin
    
    # Extract reference and current directions
    R_prev = prev_transform[:3, :3]
    R_curr = current_transform[:3, :3]
    
    # Use a reference direction perpendicular to joint axis
    # For most joints, we can use the x or y axis of the previous frame
    ref_dir = R_prev @ np.array([1, 0, 0])
    
    # Project reference direction onto plane perpendicular to joint axis
    ref_proj = ref_dir - np.dot(ref_dir, joint_axis) * joint_axis
    if np.linalg.norm(ref_proj) < 1e-6:
        ref_dir = R_prev @ np.array([0, 1, 0])
        ref_proj = ref_dir - np.dot(ref_dir, joint_axis) * joint_axis
    ref_proj = ref_proj / np.linalg.norm(ref_proj)
    
    # Current direction (rotated)
    curr_dir = R_curr @ np.array([1, 0, 0])
    curr_proj = curr_dir - np.dot(curr_dir, joint_axis) * joint_axis
    if np.linalg.norm(curr_proj) > 1e-6:
        curr_proj = curr_proj / np.linalg.norm(curr_proj)
    else:
        curr_proj = ref_proj
    
    # Compute angle between reference and current
    angle_rad = radians(angle_deg)
    
    # Create arc points
    num_points = max(20, int(abs(angle_deg) / 5))
    angles = np.linspace(0, angle_rad, num_points)
    
    # Rodrigues rotation to create arc
    arc_points = []
    for theta in angles:
        # Rotate ref_proj around joint_axis by theta
        cos_t = cos(theta)
        sin_t = sin(theta)
        rotated = (cos_t * ref_proj + 
                  sin_t * np.cross(joint_axis, ref_proj) + 
                  (1 - cos_t) * np.dot(joint_axis, ref_proj) * joint_axis)
        arc_point = joint_pos + radius * rotated
        arc_points.append(arc_point)
    
    if len(arc_points) < 2:
        return []
    
    arc_points = np.array(arc_points)
    
    # Draw arc
    arc_line, = ax.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
                       '--', color=color, linewidth=2, alpha=0.7, zorder=3)
    
    # Draw angle label at midpoint
    mid_idx = len(arc_points) // 2
    label_pos = arc_points[mid_idx] + 0.03 * (arc_points[mid_idx] - joint_pos) / np.linalg.norm(arc_points[mid_idx] - joint_pos)
    
    # Simple text positioning (approximate)
    text = ax.text(label_pos[0], label_pos[1], label_pos[2], 
                  f"{angle_deg:.1f}°", fontsize=9, color=color, 
                  fontweight='500', zorder=5)
    
    return [arc_line, text]

def draw_robot():
    global ee_text, target_marker, target_label, sliders
    global animation_active, target_angles, current_angles, animation_progress

    fig = plt.figure(figsize=(14, 10), facecolor='#f8f9fa')
    ax = fig.add_subplot(111, projection='3d', facecolor='#ffffff')
    plt.subplots_adjust(left=0.05, right=0.85, bottom=0.05, top=0.95)
    
    # Colors for each link (original colors)
    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e', '#9467bd', '#8c564b']
    link_color = '#808080'  # For joint markers
    
    link_cylinders = []
    axis_lines = []
    angle_arcs = []  # Store angle arc artists
    joint_markers = []
    ee_marker = None
    
    # Link opacity (default more transparent)
    link_opacity = 0.3  # Default opacity (was 0.6)
    opacity_slider = None
    
    # Graph view rotation angles
    view_elev = 20
    view_azim = 45
    view_roll = 0  # Note: matplotlib doesn't directly support roll, but we'll include it for future use
    
    # Store initial view for camera reset
    initial_elev = 20
    initial_azim = 45
    initial_roll = 0
    ax.view_init(elev=initial_elev, azim=initial_azim)
    
    # Store initial axis limits for zoom reset
    initial_xlim = (-0.8, 0.8)
    initial_ylim = (-0.8, 0.8)
    initial_zlim = (0, 1.2)
    
    # Set axis limits and labels
    ax.set_xlim(initial_xlim)
    ax.set_ylim(initial_ylim)
    ax.set_zlim(initial_zlim)
    ax.set_xlabel('X (m)', fontsize=12, fontweight='500', color='#2c3e50')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='500', color='#2c3e50')
    ax.set_zlabel('Z (m)', fontsize=12, fontweight='500', color='#2c3e50')
    ax.set_title('6 DOF Robot', fontsize=15, pad=10, 
                 fontweight='600', color='#2c3e50')
    
    # Set grid
    ax.grid(True, alpha=0.3, linewidth=0.5)

    def update_plot():
        global ee_text, target_marker, target_label
        nonlocal ee_marker

        # Remove old artists
        for cyl_group in link_cylinders:
            if isinstance(cyl_group, list):
                for c in cyl_group:
                    safe_remove(c)
            else:
                safe_remove(cyl_group)
        for axes in axis_lines:
            for line in axes:
                safe_remove(line)
        for arc_group in angle_arcs:
            if isinstance(arc_group, list):
                for arc in arc_group:
                    safe_remove(arc)
            else:
                safe_remove(arc_group)
        for marker in joint_markers:
            safe_remove(marker)
        safe_remove(ee_marker)
        safe_remove(ee_text)
        safe_remove(target_marker)
        safe_remove(target_label)

        link_cylinders.clear()
        axis_lines.clear()
        angle_arcs.clear()
        joint_markers.clear()
        ee_marker = None
        ee_text = None
        target_marker = None
        target_label = None

        # Compute forward kinematics (like 2D script - maintains chain consistency)
        current_theta = get_parameters()
        ee_position, joint_positions, transforms = compute_fk_full(current_theta)
        
        # Draw links as cylinders (with original colors and adjustable opacity)
        # Adjust radius per link: Link 1 bigger, Links 2-3 smaller, Links 4-5 medium
        base_radius = 0.015  # Base radius
        radius_multipliers = [1.5, 0.7, 0.7, 1.0, 1.0]  # Link 1 bigger, 2-3 smaller, 4-5 medium
        
        # NO visual extensions - keep consistent chain, links connect properly at joints
        # We have 5 actual links (Link 1 to Link 5), but positions include base and all joints
        # Track which actual link we're on (skipping zero-length links)
        actual_link_idx = 0
        for i in range(len(joint_positions) - 1):
            p1 = joint_positions[i]
            p2 = joint_positions[i + 1]
            
            # Skip if positions are the same (zero-length link, e.g., base to joint 1)
            if np.allclose(p1, p2):
                continue
            
            # Draw cylinder exactly from joint to joint (no extensions)
            # Use actual_link_idx to index into radius_multipliers (for the 5 actual links)
            multiplier_idx = min(actual_link_idx, len(radius_multipliers) - 1)
            cylinder_radius = base_radius * radius_multipliers[multiplier_idx]
            color_idx = actual_link_idx % len(colors)
            cyl_surfaces = draw_cylinder(ax, p1, p2, radius=cylinder_radius, 
                                        color=colors[color_idx], alpha=link_opacity)
            if cyl_surfaces:
                link_cylinders.append(cyl_surfaces)
            
            actual_link_idx += 1
        
        # Draw coordinate axes at each joint and end-effector (RGB: X=red, Y=green, Z=blue)
        # Show axes at base, joint 3, joint 4, joint 5, and end-effector
        key_joints = [0, 2, 3, 4, 6]  # Base, joint 3, joint 4, joint 5, end-effector
        for i, (pos, transform) in enumerate(zip(joint_positions, transforms)):
            if i in key_joints or i == len(joint_positions) - 1:  # Show at key joints and end-effector
                axes = draw_axes(ax, pos, transform, axis_length=0.04, linewidth=2.0)
                axis_lines.append(axes)
            else:
                # Add empty list to keep indexing consistent
                axis_lines.append([])
        
        # Draw angle arcs for each joint
        current_theta = get_parameters()
        arc_colors = ['#ff6b6b', '#4ecdc4', '#45b7d1', '#f9ca24', '#6c5ce7', '#a29bfe']
        
        # Draw angle arcs for each joint
        for i in range(6):
            if i == 0:
                # Base joint rotates around z-axis (world frame)
                joint_axis = np.array([0, 0, 1])
                prev_transform = np.eye(4)  # Identity (world frame)
            else:
                # Get rotation axis from previous transform
                # For UR5, joints rotate around z-axis of previous frame
                prev_transform = transforms[i]
                joint_axis = prev_transform[:3, 2]  # Z-axis of previous frame
            
            current_transform = transforms[i + 1]
            joint_pos = joint_positions[i]
            angle_val = current_theta[i]
            
            # Only draw arc if angle is significant and for first 3 joints (to reduce clutter)
            if abs(angle_val) > 5.0 and i < 3:  # Only show arcs for first 3 joints, and only if angle > 5°
                arc_artists = draw_angle_arc_3d(ax, joint_pos, prev_transform, current_transform,
                                              joint_axis, angle_val, radius=0.06, 
                                              color=arc_colors[i % len(arc_colors)])
                if arc_artists:
                    angle_arcs.append(arc_artists)
            else:
                angle_arcs.append([])
        
        # Draw joint markers (smaller, at joint centers)
        for i, pos in enumerate(joint_positions[:-1]):  # Exclude end-effector
            marker, = ax.plot([pos[0]], [pos[1]], [pos[2]], 'o', 
                            color=link_color, markersize=6,
                            markeredgecolor='white', markeredgewidth=1, zorder=4)
            joint_markers.append(marker)
        
        # Draw end-effector marker (black dot like in 2D)
        ee_marker, = ax.plot([ee_position[0]], [ee_position[1]], [ee_position[2]], 
                           'o', color='#000000', markersize=10,
                           markeredgecolor='#ffffff', markeredgewidth=2, 
                           label='End-Effector', zorder=4)
        
        # Draw base marker
        base_marker, = ax.plot([0], [0], [0], 's', color='#2c3e50', 
                             markersize=10, markeredgecolor='#1a252f', 
                             markeredgewidth=2, label='Base', zorder=4)
        joint_markers.append(base_marker)

        # Show end-effector position in MANUAL mode (near the actual end-effector, like 2D)
        # EE coordinates come from forward kinematics (same as 2D: compute_fk(get_parameters()))
        if get_mode() == Mode.MANUAL:
            # Use the user's compute_fk function to get the actual end-effector coordinates
            # This ensures the displayed coordinates match what the user's FK implementation returns
            from forward_kinematics_3d import compute_fk
            x, y, z = compute_fk(current_theta)
            
            # Find a good position for the text label near the end-effector
            # Use the direction from the second-to-last joint to end-effector
            if len(joint_positions) >= 2:
                prev_joint = joint_positions[-2]
                direction = ee_position - prev_joint
                length = np.linalg.norm(direction)
                if length > 1e-6:
                    direction = direction / length
                    # Offset perpendicular to the link direction with more space
                    # Try to find a good offset that doesn't overlap
                    offset_distance = 0.25  # Increased from 0.15 to 0.25 for more space
                    # Use a perpendicular vector (cross product with up vector)
                    up_vec = np.array([0, 0, 1])
                    perp = np.cross(direction, up_vec)
                    if np.linalg.norm(perp) < 1e-6:
                        # If parallel to up, use a different perpendicular
                        perp = np.cross(direction, np.array([1, 0, 0]))
                    perp = perp / np.linalg.norm(perp) if np.linalg.norm(perp) > 1e-6 else np.array([1, 0, 0])
                    text_pos = ee_position + perp * offset_distance
                    # Also add some offset along the direction to move it further away
                    text_pos = text_pos + direction * 0.1
                else:
                    text_pos = ee_position + np.array([0.25, 0, 0.1])
            else:
                text_pos = ee_position + np.array([0.25, 0, 0.1])
            
            # Draw text in 3D space near end-effector
            ee_text = ax.text(text_pos[0], text_pos[1], text_pos[2],
                            f"({x:.3f}, {y:.3f}, {z:.3f})",
                            fontsize=9, color='#000000', fontweight='normal',
                            fontfamily='monospace',
                            bbox=dict(facecolor='white', alpha=0.95, 
                                     edgecolor='#000000', pad=3, 
                                     linewidth=1.2, boxstyle='round,pad=0.4'),
                            zorder=10)

        # Show target in IK mode (like 2D - marked with x and label near target)
        if get_mode() == Mode.IK:
            target = get_target()
            if target is not None and len(target) == 3 and all(t is not None for t in target):
                tx, ty, tz = float(target[0]), float(target[1]), float(target[2])
                # Draw target marker (x symbol like in 2D)
                target_marker, = ax.plot([tx], [ty], [tz], 'x', 
                                       color='#000000', markersize=15,
                                       markeredgewidth=3, zorder=6)
                
                # Draw target label near the target point (like 2D: tx + 0.3, ty + 0.3)
                target_label = ax.text(tx + 0.15, ty + 0.15, tz + 0.1,
                                       f"Target: ({tx:.2f}, {ty:.2f}, {tz:.2f})",
                                       fontsize=10, color='#000000', fontweight='bold',
                                       fontfamily='monospace',
                                       bbox=dict(facecolor='white', alpha=0.95, 
                                                edgecolor='#000000', pad=3, 
                                                linewidth=1.5, boxstyle='round,pad=0.4'),
                                       zorder=11)

        # Update legend with links by color (like in 2D)
        if not hasattr(update_plot, "legend"):
            # Create custom legend entries for links
            from matplotlib.lines import Line2D
            legend_elements = []
            # Add link entries (5 links: Link 1 to Link 5, end-effector is at Joint 6)
            link_names = ['Link 1', 'Link 2', 'Link 3', 'Link 4', 'Link 5']
            for i, (name, color) in enumerate(zip(link_names, colors)):
                legend_elements.append(Line2D([0], [0], color=color, lw=3, label=name))
            # Add other elements
            legend_elements.append(Line2D([0], [0], marker='s', color='w', 
                                         markerfacecolor='#2c3e50', markersize=8, label='Base'))
            legend_elements.append(Line2D([0], [0], marker='o', color='w', 
                                         markerfacecolor='#000000', markersize=8, label='End-Effector'))
            
            ax.legend(handles=legend_elements, loc='upper left', fontsize=9, 
                     frameon=True, fancybox=True, shadow=True, framealpha=0.95, 
                     edgecolor='#bdc3c7')
            update_plot.legend = True

        fig.canvas.draw_idle()

    slider_width = 0.08
    slider_height = 0.015
    start_x = 0.87
    start_y = 0.90

    def destroy_sliders():
        global sliders
        for slider in sliders:
            if slider.ax is not None:
                try:
                    slider.ax.remove()
                except:
                    pass
        sliders.clear()

    def create_sliders():
        destroy_sliders()  

        joint_names = ['θ₁', 'θ₂', 'θ₃', 'θ₄', 'θ₅', 'θ₆']
        init = get_parameters()
        mins = [-180, -180, -180, -180, -180, -180]
        maxs = [180, 0, 180, 180, 180, 180]  # Joint 2 typically limited to [-180, 0]

        for i, (name, val, mn, mx) in enumerate(zip(joint_names, init, mins, maxs)):
            y_pos = start_y - i * (slider_height + 0.02)
            ax_slider = fig.add_axes([start_x, y_pos, slider_width, slider_height])
            
            slider = Slider(
                ax_slider, name, mn, mx,
                valinit=val,
                valfmt='%.1f',
                facecolor='none'
            )
            
            slider.label.set_fontsize(10)
            slider.label.set_fontweight('500')
            slider.label.set_color('#2c3e50')
            slider.valtext.set_fontsize(9)
            slider.valtext.set_fontweight('normal')
            slider.valtext.set_color('#2c3e50')
            slider.poly.set_alpha(0.4)
            slider.track.set_linewidth(2)
            
            sliders.append(slider)

        def on_change(val):
            new_theta = [sliders[i].val for i in range(6)]
            set_parameters(new_theta)
            update_plot()

        for s in sliders:
            s.on_changed(on_change)

    def show_sliders(show=True):
        for slider in sliders:
            if slider.ax is not None:
                slider.ax.set_visible(show)
        fig.canvas.draw_idle()

    total_slider_height = 6 * (slider_height + 0.02)
    
    # Add graph rotation sliders and opacity slider (between theta sliders and mode selection)
    # Position them right after the theta sliders
    control_y = start_y - total_slider_height - 0.05
    
    # Add opacity slider first
    opacity_y = control_y
    ax_opacity = fig.add_axes([start_x, opacity_y, slider_width, slider_height])
    opacity_slider = Slider(ax_opacity, 'Opacity', 0.1, 1.0, valinit=link_opacity, valfmt='%.2f')
    opacity_slider.label.set_fontsize(10)
    opacity_slider.label.set_fontweight('500')
    opacity_slider.label.set_color('#2c3e50')
    opacity_slider.valtext.set_fontsize(9)
    opacity_slider.valtext.set_fontweight('normal')
    opacity_slider.valtext.set_color('#2c3e50')
    opacity_slider.poly.set_alpha(0.4)
    opacity_slider.track.set_linewidth(2)
    
    def on_opacity_change(val):
        nonlocal link_opacity
        link_opacity = val
        update_plot()
    
    opacity_slider.on_changed(on_opacity_change)
    
    # Add graph rotation sliders (Elevation, Azimuth)
    rot_sliders = []
    rot_names = ['Elev', 'Azim']
    rot_values = [view_elev, view_azim]
    rot_ranges = [(-90, 90), (0, 360)]  # Elevation: -90 to 90, Azimuth: 0 to 360
    
    for i, (name, val, (r_min, r_max)) in enumerate(zip(rot_names, rot_values, rot_ranges)):
        y_pos = opacity_y - (i + 1) * (slider_height + 0.02)
        ax_rot = fig.add_axes([start_x, y_pos, slider_width, slider_height])
        rot_slider = Slider(ax_rot, f'Rot {name}', r_min, r_max, valinit=val, valfmt='%.1f')
        rot_slider.label.set_fontsize(10)
        rot_slider.label.set_fontweight('500')
        rot_slider.label.set_color('#2c3e50')
        rot_slider.valtext.set_fontsize(9)
        rot_slider.valtext.set_fontweight('normal')
        rot_slider.valtext.set_color('#2c3e50')
        rot_slider.poly.set_alpha(0.4)
        rot_slider.track.set_linewidth(2)
        rot_sliders.append(rot_slider)
    
    def on_rot_change(val, axis_idx):
        nonlocal view_elev, view_azim
        if axis_idx == 0:
            view_elev = val
        else:
            view_azim = val
        ax.view_init(elev=view_elev, azim=view_azim)
        fig.canvas.draw_idle()
    
    for i, slider in enumerate(rot_sliders):
        def make_callback(idx):
            return lambda val: on_rot_change(val, idx)
        slider.on_changed(make_callback(i))
    
    # Mode selection (with spacing from rotation sliders)
    mode_y = opacity_y - 3 * (slider_height + 0.02) - 0.08
    ax_mode = fig.add_axes([start_x - 0.015, mode_y, slider_width + 0.04, 0.08])
    radio = RadioButtons(ax_mode, ('Forward Kinematics', 'Inverse Kinematics'), active=0)
    for text in radio.labels:
        text.set_fontsize(9)
    
    # Add reset buttons (with spacing)
    button_y = mode_y - 0.12
    button_height = 0.04
    button_width = slider_width + 0.04
    
    # Reset robot pose button
    ax_reset_pose = fig.add_axes([start_x - 0.015, button_y, button_width, button_height])
    btn_reset_pose = Button(ax_reset_pose, 'Reset Pose', color='#e74c3c', hovercolor='#c0392b')
    btn_reset_pose.label.set_fontsize(9)
    btn_reset_pose.label.set_color('white')
    btn_reset_pose.label.set_fontweight('bold')
    
    # Reset camera view button
    ax_reset_view = fig.add_axes([start_x - 0.015, button_y - button_height - 0.02, button_width, button_height])
    btn_reset_view = Button(ax_reset_view, 'Reset View', color='#3498db', hovercolor='#2980b9')
    btn_reset_view.label.set_fontsize(9)
    btn_reset_view.label.set_color('white')
    btn_reset_view.label.set_fontweight('bold')
    
    def reset_robot_pose(event):
        """Reset robot to initial UR5 home position."""
        set_parameters(INITIAL_THETA.copy())
        # Update sliders if they exist and are visible
        for i, slider in enumerate(sliders):
            if slider.ax is not None and slider.ax.get_visible():
                slider.set_val(INITIAL_THETA[i])
        update_plot()
    
    def reset_camera_view(event):
        """Reset 3D camera view to initial position and zoom."""
        nonlocal view_elev, view_azim
        ax.view_init(elev=initial_elev, azim=initial_azim)
        # Reset rotation angles
        view_elev = initial_elev
        view_azim = initial_azim
        # Reset sliders
        rot_sliders[0].set_val(initial_elev)
        rot_sliders[1].set_val(initial_azim)
        # Reset zoom by restoring initial axis limits
        ax.set_xlim(initial_xlim)
        ax.set_ylim(initial_ylim)
        ax.set_zlim(initial_zlim)
        fig.canvas.draw_idle()
    
    btn_reset_pose.on_clicked(reset_robot_pose)
    btn_reset_view.on_clicked(reset_camera_view)

    def on_mode_change(label):
        global animation_active, target_angles, current_angles
        
        new_mode = Mode.MANUAL if label == 'Forward Kinematics' else Mode.IK
        set_mode(new_mode)
        
        animation_active = False
        target_angles = None
        current_angles = None

        if new_mode == Mode.IK:
            set_parameters(INITIAL_THETA.copy())
            show_sliders(False)
            set_target(None, None, None)
            # Disable matplotlib's default navigation tools in IK mode to prevent graph movement
            ax.set_navigate(False)
            # Disable toolbar pan/zoom by setting mode to None
            toolbar = fig.canvas.toolbar
            if toolbar is not None:
                toolbar.set_message('Click to set target (IK mode)')
                # Save current mode and set to None to disable pan/zoom
                if not hasattr(toolbar, '_saved_mode'):
                    toolbar._saved_mode = toolbar.mode
                toolbar.mode = ''
        else:
            create_sliders()     
            show_sliders(True)
            # Re-enable navigation in MANUAL mode
            ax.set_navigate(True)
            # Restore toolbar mode if it was saved
            toolbar = fig.canvas.toolbar
            if toolbar is not None and hasattr(toolbar, '_saved_mode'):
                toolbar.mode = toolbar._saved_mode   

        global target_marker, target_label
        safe_remove(target_marker)
        safe_remove(target_label)
        target_marker = None
        target_label = None

        update_plot()

    radio.on_clicked(on_mode_change)

    # Animation function removed - user will implement their own IK and animation if needed
    # The animation_step function below can be used for smooth transitions if needed

    def animation_step(frame):
        global animation_active, target_angles, current_angles, animation_progress
        
        if not animation_active or target_angles is None:
            return
        
        animation_progress += animation_speed
        
        if animation_progress >= 1.0:
            set_parameters(target_angles)
            animation_active = False
            target_angles = None
            current_angles = None
            animation_progress = 0
            update_plot()
        else:
            t = animation_progress
            t = t * t * (3 - 2 * t)  # Smooth step interpolation
            
            interpolated = [
                current_angles[i] + (target_angles[i] - current_angles[i]) * t
                for i in range(6)
            ]
            set_parameters(interpolated)
            update_plot()

    def on_click(event):
        global animation_active
        
        if get_mode() != Mode.IK or event.inaxes != ax:
            return
        
        if animation_active:
            return
        
        # Only handle left mouse button clicks
        if event.button != 1:
            return
        
        # Get the mouse position
        if event.xdata is None or event.ydata is None:
            return
        
        # Get current end-effector position for z reference
        current_theta = get_parameters()
        from forward_kinematics_3d import compute_fk
        ee_x, ee_y, ee_z = compute_fk(current_theta)
        reference_z = float(ee_z) if ee_z is not None else 0.5
        
        # Get axis limits
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        zlim = ax.get_zlim()
        
        # Use proj3d to convert click to 3D coordinates - optimized version
        from mpl_toolkits.mplot3d import proj3d
        
        # Get the mouse position in display coordinates
        mouse_x = event.x
        mouse_y = event.y
        
        # Optimized approach: Start with reference z, do 2D search in x-y plane
        # Then optionally refine in z if needed
        best_point = None
        min_dist = float('inf')
        
        # First, search in x-y plane at the reference z (much faster - 2D search)
        # Use a reasonable grid size for speed
        x_samples = np.linspace(xlim[0], xlim[1], 30)
        y_samples = np.linspace(ylim[0], ylim[1], 30)
        
        for x_test in x_samples:
            for y_test in y_samples:
                # Project this 3D point to 2D screen coordinates at reference z
                x2d, y2d = proj3d.proj_transform(x_test, y_test, reference_z, ax.get_proj())[:2]
                
                # Convert from axes coordinates to display coordinates
                point_display = ax.transData.transform([[x2d, y2d]])[0]
                
                # Calculate distance to mouse click
                dist = np.sqrt((point_display[0] - mouse_x)**2 + 
                             (point_display[1] - mouse_y)**2)
                
                if dist < min_dist:
                    min_dist = dist
                    best_point = [x_test, y_test, reference_z]
        
        # Optional: Quick refinement in z if the initial search found a good match
        # Only do this if we found a reasonable match (within 50 pixels)
        if best_point is not None and min_dist < 50:
            # Refine z around the reference value
            z_refine_samples = np.linspace(max(zlim[0], reference_z - 0.2), 
                                         min(zlim[1], reference_z + 0.2), 5)
            for z_test in z_refine_samples:
                x_test, y_test = best_point[0], best_point[1]
                x2d, y2d = proj3d.proj_transform(x_test, y_test, z_test, ax.get_proj())[:2]
                point_display = ax.transData.transform([[x2d, y2d]])[0]
                dist = np.sqrt((point_display[0] - mouse_x)**2 + 
                             (point_display[1] - mouse_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_point = [x_test, y_test, z_test]
        
        if best_point is not None:
            click_x, click_y, click_z = best_point
            
            # Clamp to axis limits
            click_x = np.clip(click_x, xlim[0], xlim[1])
            click_y = np.clip(click_y, ylim[0], ylim[1])
            click_z = np.clip(click_z, zlim[0], zlim[1])
            
            # Set the target
            set_target(click_x, click_y, click_z)
            update_plot()

    def on_key_press(event):
        """Handle keyboard input for setting target in IK mode."""
        if get_mode() != Mode.IK:
            return
        
        if animation_active:
            return
        
        # Use arrow keys and page up/down to adjust target position
        target = get_target()
        if target is None or len(target) != 3 or any(t is None for t in target):
            target = [0.3, 0.0, 0.5]
        else:
            target = [float(t) if t is not None else (0.3 if i == 0 else (0.0 if i == 1 else 0.5)) 
                     for i, t in enumerate(target)]
        
        step = 0.05
        if event.key == 'up':
            target[2] += step
        elif event.key == 'down':
            target[2] -= step
        elif event.key == 'left':
            target[0] -= step
        elif event.key == 'right':
            target[0] += step
        elif event.key == 'pageup':
            target[1] += step
        elif event.key == 'pagedown':
            target[1] -= step
        elif event.key == 'r' or event.key == 'R':
            # Reset to default position
            target = [0.3, 0.0, 0.5]
        
        # Ensure all values are valid numbers before clamping
        target = [float(t) for t in target]
        
        # Clamp to reasonable bounds
        target[0] = np.clip(target[0], -0.8, 0.8)
        target[1] = np.clip(target[1], -0.8, 0.8)
        target[2] = np.clip(target[2], 0.1, 1.1)
        
        # Just set the target visually - no IK solving here
        # User will implement their own IK and call it separately
        set_target(target[0], target[1], target[2])
        update_plot()

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    try:
        fig.canvas.manager.set_window_title('6 DOF Robot Kinematics')
    except:
        pass

    anim = FuncAnimation(fig, animation_step, interval=20, blit=False, cache_frame_data=False)

    create_sliders()
    show_sliders(True)
    update_plot()
    plt.show()