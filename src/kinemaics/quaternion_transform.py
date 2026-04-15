# Quaternion_Transform : A 4x4 Homogeneous Transformation Math Engine using Quaternions  

import numpy as np

"""
Quaternion Transformation Functions :

    - transform_vector():
        This function takes an input in one coordinate frame, outupts it in another 

    - transform_rates()
        This function takes an input rate and pose of the initial frame and then gives the rates in final coordinate frame.
"""

def transform_vector(vec_in, pose_in, trans_in):

    """
    transform_vector: This function takes Vector, Rotation Vector, and Translation Vector in one frame. Outputs it in another coordinate frame.
        Input:
            - vec_in (x,y,z) : Input Vector
            - pose_in (ox,oy,oz) : Rotation Vector
            - trans_in (tx,ty,tz) : Translation Vector

        Output:
            - np.arr(3) : Transformed Vector
    """

    # Input Vector (Homogeneous)
    vec_in_hom = np.append(vec_in,1)

    # Magnitude of Anlgles through each axis
    o = np.linalg.norm(pose_in)

    # If Rotation is zero: return old vector
    if o < 1e-9:
        return vec_in + trans_in
    
    # Scaling the each Rotation Vector such that sum of square is 1.
    pose_scaled = pose_in/o
    ox_scaled  = pose_scaled[0]
    oy_scaled = pose_scaled[1]
    oz_scaled =  pose_scaled[2]
   
    # Defining Trigonometric Functions
    C =  np.cos(o)
    S =  np.sin(o)
    V = 1 - C

    # Elements of the Matrix    
    r11 = C + (ox_scaled**2)* V 
    r12 = ox_scaled * oy_scaled * V - oz_scaled * S
    r13 = ox_scaled * oz_scaled * V + oy_scaled * S
    # Translation
    r14 = trans_in[0]

    r21 = ox_scaled * oy_scaled * V + oz_scaled * S
    r22 = C + oy_scaled**2 * V 
    r23 = oy_scaled * oz_scaled * V - ox_scaled * S
    # Translation
    r24 = trans_in[1] 

    r31 = ox_scaled * oz_scaled * V - oy_scaled * S 
    r32 = oy_scaled * oz_scaled * V + ox_scaled * S
    r33 = C + oz_scaled**2 * V 
    # Translation 
    r34 = trans_in[2]

    R = np.array([
        [r11, r12, r13, r14],
        [r21, r22, r23, r24],
        [r31, r32, r33, r34],
        [0, 0, 0, 1]
        ])

    # New Vector Updated by Matrix Mult between the Rotation Matrix and teh Old Vector
    vector_new = R @ vec_in_hom
    
    # Return the new_transformed non-homogeneous vector
    return vector_new[:3]



def transform_rate(rate_in, pose_in):
    """
    transform_rate: Transforms Rates from one frame to another coordiante frame.

    Input:
        - rate_in (rx, ry, rz): Rates in the initial frame. 
        - pose_in (ox, oy, oz): Pose of the initial frame to the final frame.
    """
    # Normalised Pose for Quaternions
    o = np.linalg.norm(pose_in)

    # No Rotation Check 
    if o < 1e-9:
        return rate_in 

    # Temporary Variable 
    S =  np.sin(o/2)

    # Defining Quaternions
    qr = np.cos(o/2)
    qi = pose_in[0]/o*S
    qj = pose_in[1]/o*S
    qk = pose_in[2]/o*S   

    # Define the vector part of Quaternions
    q_vec = np.array([qi, qj, qk]) 

    # Compute new Vector
    inner_term = np.cross(q_vec, rate_in)
    outer_term = np.cross(q_vec, inner_term + qr*rate_in)
    rate_out = rate_in + 2 * outer_term 

    return rate_out






