# Contains Rotations Functions 

import numpy as np

# Rotation About X-Axis (Roll)
def rot_X(roll):

    # Defining the Trig Functions
    c = np.cos(roll)
    s = np.sin(roll)

    # Rotation Matrix for Rotation About X (Roll)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

# Rotation About Y-Axis (Pitch)
def rot_Y(pitch):

    # Defining the Trig Functions 
    c = np.cos(pitch)
    s = np.sin(pitch)

    # Rotation Matrix for Rotation About Y (Pitch)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# Rotation About Z-Axis (Yaw)
def rot_Z(yaw):

    # Defining the Trig Functions 
    c = np.cos(yaw)
    s = np.sin(yaw)

    # Rotation Matrix for Rotation About Z (Yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

# Euler Rotation Matrix (ZYX Convention) 
def rotation(roll, pitch, yaw):
    return rot_Z(yaw) @ rot_Y(pitch) @ rot_X(roll)

def aero_forces_to_body(F_inst, alpha_0, theta):
    """
    Transforms the instantaneous aerodynamic force into Thrust, Lift, and Yaw centripetal forces.
    Equation 9 from Mao 2024 implementation.md
    
    [F_T, -F_L, F_C]^T = rot_Y(-alpha_0) * rot_X(theta) * [0, 0, F_inst]^T
    """
    input_vector = np.array([0.0, 0.0, F_inst])
    
    # Apply rotation matrices
    rotated_x = rot_X(theta) @ input_vector
    rotated_y = rot_Y(-alpha_0) @ rotated_x
    
    # Return directly (F_T, -F_L, F_C)
    return rotated_y

def camera_to_wing_transform(p_c, T_cw, theta, alpha_0, k_Lambda):
    """
    Measurements & Vision Coordinate Transformation
    Equation 29-32 from Mao 2024
    """
    p_c = np.array(p_c, dtype=float)
    T_cw = np.array(T_cw, dtype=float)
    
    R_cw = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    val = k_Lambda / np.cos(alpha_0)
    k_cw = np.diag([val, val])
    
    return k_cw @ (R_cw @ (p_c + T_cw))
