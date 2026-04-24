# Frame Transform : Contains all transformation between the Coordinate Frames 

import numpy as np
from .quaternion_transform import transform_vector, transform_rate
"""quaternion_transform.py:
    Frames: 
        
        - Inertial Frame: The NED convention
        - Body Frame: The Local Coordinate Frame, defined for the body
            - Hinge Frame: The Coordinate frame dedicated to the hinge for flapping wing uav 
                - Wing Frame: The Frame set for the wing strips for bet
            - Tail Frame: The Frame dedicated to the tail and it's moment and forces.

    Frame_Transform:
        - To define transformation between the frames
    
    Functions: 
        - vector_Transform_Inertial_to_Body: A function which takes a vector and transforms from Inertial Frame to Body Frame
        - vector_Transform_Body_to_Tail: A function which takes a vector and transforms from Body Frame to Tail Frame 
        - vector_Transform_Body_to_Wing: A function which takes a vector and transforms from Body Frame to Wing Frame 

"""



# Vector Transformation from Parent to Child 
def vector_Transform_Inertial_to_Body(vec_in, pose_in):
    """
        vector_Transform_Inertial_to_Body: To Transform a Vector from Inertial frame to Body frame.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - pose_in(6-elem): Global Pose 
             Output:
                - vec_out(np.array(3)): Output vector in the output frame.
    """
    # Transform Vectors from Inertial Frame to Body Frame
    rotation = pose_in[3:6]
    trans_array = np.zeros(3) # As we transforming vectors

    return transform_vector(vec_in, rotation, trans_array)



# State Transform from Parent to Child

def state_Transform_Body_to_Winghinge(state_in, pose_in, flap_in):
    """
    state_Transform_Body_to_Winghinge: Transforms full 6-DOF State from Body to Winghinge.
    """
    vel_in = state_in[:3]
    rate_in = state_in[3:6]
    r_offset = pose_in[:3]
    
    # Structural mounting rotation + active flapping rotation (Stroke acts on local X-axis)
    rotation = pose_in[3:6].copy()
    rotation[0] += flap_in[0]
    
    # Linear Velocity Transfer 
    v_swing_body = vel_in + np.cross(rate_in, r_offset)
    vel_out = transform_vector(v_swing_body, rotation, np.zeros(3))
    
    # Angular Rate Transfer
    rate_out = transform_rate(rate_in, rotation)
    rate_out[0] += flap_in[1]  # += flap_rate (addition of angular velocities)
    
    return np.concatenate([vel_out, rate_out])

def state_Transform_Winghinge_to_Wingstrip(state_in, pose_in):
    """
    state_Transform_Winghinge_to_Wingstrip: Transforms full 6-DOF State from Hinge to Strip.
    """
    vel_in = state_in[:3]
    rate_in = state_in[3:6]
    r_offset = pose_in[:3]
    rotation = pose_in[3:6]
    
    v_swing_hinge = vel_in + np.cross(rate_in, r_offset)
    vel_out = transform_vector(v_swing_hinge, rotation, np.zeros(3))
    rate_out = transform_rate(rate_in, rotation)
    
    return np.concatenate([vel_out, rate_out])

def state_Transform_Body_to_Tailhinge(state_in, pose_in):
    """
    Transforms 6-DOF State from Body Center of Mass to the Tail Hinge.
    """
    vel_in = state_in[:3]
    rate_in = state_in[3:6]
    r_offset = pose_in[:3]
    rotation = pose_in[3:6]
    
    v_swing = vel_in + np.cross(rate_in, r_offset)
    vel_out = transform_vector(v_swing, rotation, np.zeros(3))
    rate_out = transform_rate(rate_in, rotation)
    
    return np.concatenate([vel_out, rate_out])

def state_Transform_Tailhinge_to_TailCOP(state_in, pose_in, deflection_in):
    """
    Transforms 6-DOF State from the Tail Hinge to the Center of Pressure.
    Applies Elevator (pitch/Y) and Rudder (yaw/Z) servo deflections.
    """
    vel_in = state_in[:3]
    rate_in = state_in[3:6]
    r_offset = pose_in[:3]
    
    # Apply surface deflection twist
    rotation = pose_in[3:6].copy()
    rotation[1] += deflection_in[0]  # Elevator (Pitch)
    rotation[2] += deflection_in[1]  # Rudder (Yaw)
    
    v_swing = vel_in + np.cross(rate_in, r_offset)
    vel_out = transform_vector(v_swing, rotation, np.zeros(3))
    rate_out = transform_rate(rate_in, rotation) # Applies surface deflection twist
    
    return np.concatenate([vel_out, rate_out])
 


# Vector Transform from Child to Parent


def wrench_Transform_Body_to_Inertial(wrench_in, pose_in):
    """
    wrench_Transform_Body_to_Inertial: Transforms the total Force and Moment 
    from the Body Frame back to the Inertial (NED) Frame.
    """
    force_in = wrench_in[:3]
    moment_in = wrench_in[3:6]
    rotation = pose_in[3:6]
    
    force_out = transform_vector(force_in, -rotation, np.zeros(3))
    moment_out = transform_vector(moment_in, -rotation, np.zeros(3))
    
    return np.concatenate([force_out, moment_out])



def wrench_Transform_Winghinge_to_Body(wrench_in, pose_in, flap_in):
    """
    wrench_Transform_Winghinge_to_Body: Transforms the total Wrench from the Hinge back to the Body Center of Mass.
    """
    force_in = wrench_in[:3]
    moment_in = wrench_in[3:6]
    r_offset = pose_in[:3]
    
    # Reconstruct the forward rotation, then invert for reverse transform
    rotation = pose_in[3:6].copy()
    rotation[0] += flap_in[0]

    force_out = transform_vector(force_in, -rotation, np.zeros(3))
    moment_rotated = transform_vector(moment_in, -rotation, np.zeros(3))

    moment_coupled = np.cross(r_offset, force_out)
    moment_out = moment_rotated + moment_coupled

    return np.concatenate([force_out, moment_out])



def wrench_Transform_Wingstrip_to_Winghinge(wrench_in, pose_in):
    """
    wrench_Transform_Wingstrip_to_Winghinge: Transform Wrench(Force, Momemnt) from Wing strip to hinge. 
    """
    force_in = wrench_in[:3]
    moment_in = wrench_in[3:6]
    r_offset = pose_in[:3]
    rotation = pose_in[3:6]

    f_hinge = transform_vector(force_in, -rotation, np.zeros(3))
    m_rotated = transform_vector(moment_in, -rotation, np.zeros(3))

    m_coupled = np.cross(r_offset, f_hinge)
    m_hinge = m_rotated + m_coupled

    return np.concatenate([f_hinge, m_hinge])



def wrench_Transform_TailCOP_to_Tailhinge(wrench_in, pose_in, deflection_in):
    """
    wrench_Transform_TailCOP_to_Tailhinge: Rotates aerodynamics forces from the deflected Tail CoP back to the Hinge.
    """
    force_in = wrench_in[:3]
    moment_in = wrench_in[3:6]
    r_offset = pose_in[:3]
    
    rotation = pose_in[3:6].copy()
    rotation[1] += deflection_in[0]
    rotation[2] += deflection_in[1]
    
    f_hinge = transform_vector(force_in, -rotation, np.zeros(3))
    m_rotated = transform_vector(moment_in, -rotation, np.zeros(3))
    
    m_coupled = np.cross(r_offset, f_hinge)
    m_hinge = m_rotated + m_coupled
    
    return np.concatenate([f_hinge, m_hinge])

def wrench_Transform_Tailhinge_to_Body(wrench_in, pose_in):
    """
    wrench_Transform_Tailhinge_to_Body: Rotates the hinge forces back to the Body CoM.
    """
    force_in = wrench_in[:3]
    moment_in = wrench_in[3:6]
    r_offset = pose_in[:3]
    rotation = pose_in[3:6]
    
    f_out = transform_vector(force_in, -rotation, np.zeros(3))
    m_rotated = transform_vector(moment_in, -rotation, np.zeros(3))
    
    m_coupled = np.cross(r_offset, f_out)
    m_out = m_rotated + m_coupled
    
    return np.concatenate([f_out, m_out])



# Rate transform from Body to Parent

def state_Transform_Body_to_Inertial(state_in, pose_in):
    """
    state_Transform_Body_to_Inertial: Transforms full 6-DOF State from Body to Inertial.
    """
    vel_in = state_in[:3]
    rate_in = state_in[3:6]
    rotation = pose_in[3:6]
    
    vel_out = transform_vector(vel_in, -rotation, np.zeros(3))
    rate_out = transform_rate(rate_in, -rotation)
    
    return np.concatenate([vel_out, rate_out])


# Wrench Summation and Abstraction Utilities

def sum_wing_strip_wrenches(strip_objects):
    """
    sum_wing_strip_wrenches: Iterates through Wing_Strip objects, transforms their local wrenches to the Winghinge, and sums them.
    """
    wrench_hinge = np.zeros(6)
    for strip in strip_objects:
        wrench_hinge += wrench_Transform_Wingstrip_to_Winghinge(strip.pull_wrench, strip.pull_pose)
    return wrench_hinge

def mirror_wing_wrench(wrench_left):
    """
    mirror_wing_wrench: Mirrors the left wing wrench to the right wing by flipping Y-axis dependent components (Lateral Force, Roll, Yaw).
    """
    return np.array([
        wrench_left[0],  # Fx (unchanged)
       -wrench_left[1],  # Fy (flipped)
        wrench_left[2],  # Fz (unchanged)
       -wrench_left[3],  # Mx (flipped)
        wrench_left[4],  # My (unchanged)
       -wrench_left[5]   # Mz (flipped)
    ])
