# Frame Transform : Contains all transformation between the Coordinate Frames 

import numpy as np
from quaternion_transform import transform_vector, transform_rate
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
                - pose_in(ox, oy, oz): Pose in
             Output:
                - vec_out(np.array(3)): Output vector in the output frame.
    """
    # Transform Vectors from Inertial Frame to Body Frame
    trans_array = np.zeros(3) # As we transforming vectors

    return transform_vector(vec_in, pose_in, trans_array)




def vector_Transform_Body_to_Tail(vec_in, elevator_deflection, rudder_deflection):

    """
        vector_Transform_Body_to_Tail: To Transform a Vector from Body frame to Tail.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - elevator_deflection(oe): Change of elevator control surface in radians.
                - rudder_deflection(or): Change of rudder control surface in radians.
           Output:
                    - vec_out(np.array(3)): Output vector in the output frame.
    """
    # Transformation Array
    tail_cop_pose = np.array([ 0.0, elevator_deflection, rudder_deflection])  
    tail_cop_offset = np.zeros(3) 

    return transform_vector(vec_in, tail_cop_pose, tail_cop_offset)



def vector_Transform_Body_to_Winghinge(vec_in, flapping_angle, wing_incidence_angle):
     """
        vector_Transform_Body_to_Winghinge: To Transform a Vector from Body to Winghinge
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - flapping_angle(of): The angle the flapping wing is oriented at the present time around the x-axis.
             Output:
                - vec_out(np.array(3)): Output vector in the output frame.
    """

    # Transformation to Wing Hinge Frame
    wing_hinge_pose = np.array([flapping_angle, wing_incidence_angle, 0.0]) 
    wing_hinge_offset = np.zeros(3)

    return transform_vector(vec_in, wing_hinge_pose, wing_hinge_offset)


    
def vector_Transform_Winghinge_to_Wingstrip(vec_in, twist_chordwise, bend_spanwise):
   """
        vector_Transform_Winghinge_to_Wingstrip: To Transform a Vector from Winghinge to wingstrip.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - twist_chordwise(ofc): Each wing strip of BET twisting w.r.t. to y-axis.
                - bend_chordwise(obc): The wing span binding along the wingspan due to incoming and apparent mass forces w.r.t. to x-axis.
             Output:
                - vec_out(np.array(3)): Output vector in the output frame.

    """
    # Transformation to Wing Strip Frame
    wing_strip_pose = np.array([bend_spanwise, twist_chordwise, 0.0]) 
    wing_strip_offset = np.zeros(3)

    return transform_vector(vec_in, wing_strip_pose, wing_strip_offset)



# Rate Trnasform from Parent to Child 

def velocity_Transform_Body_to_Winghinge(vel_in, rate_in, pose_in, r_offset):
     """
        velocity_Transform_Body_to_Winghinge: To Transform the velocity rates from Body to Inertial.
            Input: 
                - vel_in(vx, vy, vz): Input Velcity of body frame.
                - rate_in(rx, ry, rz): Input Rate.
                - pose_in(ox, oy, oz): Output frame's pose w.r.t. input frame
                - r_offset(rx, ry, rz): Leverarm between the com and the hinge
         Output:
                - vel_out(np.array(3)): Output velocity in the output frame.
    """
    v_swing_body = vel_in + np.cross(rate_in, r_offset)
    return transform_vector(v_swing_body, pose_in, np.zeros(3))
 


def velocity_Transform_Winghinge_to_Wingstrip(vel_in, rate_in, pose_in, r_offset):
     """
        velocity_Transform_Winghinge_to_Wingstrip: To Transform the velocity rates from Body to Inertial.
            Input: 
                - vel_in(vx, vy, vz): Input Velcity of body frame.
                - rate_in(rx, ry, rz): Input Rate.
                - pose_in(ox, oy, oz): Output frame's pose w.r.t. input frame
                - r_offset(rx, ry, rz): Leverarm between the com and the hinge
         Output:
                - vel_out(np.array(3)): Output velocity in the output frame.
    """
    v_swing_body = vel_in + np.cross(rate_in, r_offset)
    return transform_vector(v_swing_body, pose_in, np.zeros(3))
 
 


def angular_rate_Transform_Body_to_Winghinge(rate_in, wing_incidence_angle, flapping_angle, flapping_rate):
     """
        angular_rate_Transform_Body_to_Winghinge: To Transform the angular rates from Body to Inertial.
            Input: 
                - rate_in(rx, ry, rz): Input Rate.
                - flapping_angle(ofp): Input angle of flapping wing around x-axis.   
         Output:
                - rate_out(np.array(3)): Output rates in the output frame.
    """
   
    # Wing Hinge Pose
    pose_arr = np.array([ flapping_angle, wing_incidence_angle, 0.0 ])
    rate_out = transform_rate(rate_in, pose_arr)
    rate_out[0] += flapping_rate 
    return rate_out 
   

def rate_Transform_Winghinge_to_Wingstrip(rate_in, twist_chordwise, bend_spanwise):
  """
        vector_Transform_Winghinge_to_Wingstrip: To Transform a Vector from Winghinge to wingstrip.
            Input: 
                - rate_in(rx, ry, rz): Input Rate
                - twist_chordwise(ofc): Each wing strip of BET twisting about y-axis.
                - bend_spanwise(obc): The wing span binding along the wingspan due to incoming and apparent mass forces about x-axis.
           Output:
                - rate_out(np.array(3)): Output rates in the output frame.
    """
    pose_in = np.array([bend_spanwise, twist_chordwise, 0.0])
    return transform_rate(rate_in, pose_in)
 


# Vector Transform from Child to Parent


def wrench_Transform_Body_to_Inertial(force_in, moment_in, pose_body):
    """
    wrench_Transform_Body_to_Inertial: Transforms the total Force and Moment 
    from the Body Frame back to the Inertial (NED) Frame.
    
    Input:
         - force_in (np.array): Total summed Force in Body Frame.
         - moment_in (np.array): Total summed Moment in Body Frame.
         - pose_body (np.array): The rotation vector. 
    Output: 
        - force_out (np.array), moment_out (np.array)
    """
    force_out = transform_vector(force_in, -pose_body, np.zeros(3))
    moment_out = transform_vector(moment_in, -pose_body, np.zeros(3))
    
    return force_out, moment_out



def wrench_Transform_Winghinge_to_Body(force_in, moment_in, flapping_angle, wing_incidence_angle, r_offset):
    """
    wrench_Transform_Winghinge_to_Body: Transforms the total Wrench from the Hinge back to the Body Center of Mass.
    
    Input:
         - force_in: Total summed Force in the Hinge Frame.
         - moment_in: Total summed Moment in the Hinge Frame.
         - flapping_angle: Current flapping orientation (around X).
         - wing_incidence_angle: Fixed or variable wing tilt (around Y).
         - r_offset: Vector FROM Body CoM TO Hinge pivot (expressed in BODY frame).

    Output: 
        - force_out (np.array), moment_out (np.array)
    """

    # pose_in
    pose_hinge = np.array([ flapping_angle, wing_incidence_angle, 0.0 ])
    
    # Wrench Rotation
    force_out = transform_vector(force_in, -pose_hinge, np.zeros(3))
    moment_rotated = transform_vector(moment_in, -pose_hinge, np.zeros(3))

    # Finding the Moment from force
    moment_coupled = np.cross(r_offset, force_out)
    moment_out = moment_rotated + moment_coupled

    return force_out, moment_out



def wrench_Transform_Wingstrip_to_Winghinge(force_in, moment_in, bend_spanwise, twist_chordwise, r_offset):
    """
    wrench_Transform_Wingstrip_to_Winghinge: Transform Wrench(Force, Momemnt) from Wing strip to hinge. 
    Input:
         - force_in(fx, fy, fz): Input Force Vector in WingStripFrame
         - moment_in(mx, my, mz): Input Moment Vector in WingStrip Frame
         - pose_in(ox, oy, oz): Pose in

    Output: 
        - force_out (np.array(3)), moment_out (np.array(3))
    """

    pose_in = np.array([ bend_spanwise, twist_chordwise, 0.0 ])

    f_hinge = transform_vector(force_in, -pose_in, np.zeros(3))
    m_out = transform_vector(moment_in, -pose_in, np.zeros(3))

    m_coupled = np.cross(r_offset, f_hinge)

    m_out = m_out + m_coupled

    return f_hinge, m_out 



def wrench_Transform_Tail_to_Body(force_in, moment_in, elevator_deflection, rudder_deflection):
    """
    wrench_Transform_Tail_to_Body: Transform Wrench(Forces, Moments) from Tail to Body frame.
    
    Inputs:
        - force_in: Force at Tail CoP (Tail Frame)
        - moment_in: Moment at Tail CoP (Tail Frame)
        - elevator_deflection: Rotation around Y
        - rudder_deflection: Rotation around Z

    Output: 
        - force_out(np.array(3)), moment_out(np.array(3))

    """

    # Pose_in (COP -> Hinge)
    r_cop_to_hinge = np.array([-0.1, 0.0, 0.0]) 
    pose_tail = np.array([0.0, elevator_deflection, rudder_deflection])

    # Wrench Rotation frrom COP to Hinge
    f_hinge = transform_vector(force_in, -pose_tail, np.zeros(3))
    m_rotated_hinge = transform_vector(moment_in, -pose_tail, np.zeros(3))

    # Moment at hinge 
    m_hinge = m_rotated_hinge + np.cross(r_cop_to_hinge, f_hinge)

    # Pose_in (Hinge -> Body)
    r_hinge_to_com = np.array([-0.3, 0.0, -0.1])
    
    f_out = f_hinge 
    
    # Moment at body 
    m_out = m_hinge + np.cross(r_hinge_to_com, f_out)

    return f_out, m_out



# Rate transform from Body to Parent

def rate_Transform_Wingstrip_to_Winghinge(vec_in, twist_chordwise, bend_spanwise):
    """
        rate_Transform_Wingstrip_to_Winghinge: To Transform the rates from Wingstrip to Winghinge.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - twist_chordwise(ofc): Each wing strip of BET twisting w.r.t. to y-axis.
                - bend_chordwise(obc): The wing span binding along the wingspan due to incoming and apparent mass forces w.r.t. to x-axis.
   
           Output:
                - rate_out(np.array(3)): Output rates in the output frame.
    """
    
    # Transformation to Wing hing Frame
    wing_strip_pose = np.array([bend_spanwise, twist_chordwise, 0.0]) 

    return transform_rate(vec_in, -wing_strip_pose)



def rate_Transform_Body_to_Inertial(rate_in, pose_in):
    """
        rate_Transform_Body_to_Inertial: To Transform the rates from Body to Inertial.
            Input: 
                - rate_in(vx, vy, vz): Input Rates
                - pose_in(ox, oy, oz): Input pose of the coordinate frame of Body w.r.t. to Inertal Frame.  
           Output:
                - rate_out(np.array(3)): Output rates in the output frame.
    """
     
    return transform_rate(rate_in, -pose_in)

# ACHRIVE IMPO INFROMATION 
# def vector_Transform_Body_to_Tail(vec_in, elevator_deflection, rudder_deflection):
#
#     # Transform to Hinge Tail
#
#     # Transformation Array
#     tail_hinge_offset = np.array([ -0.3, 0.0, -0.1])
#     tail_hinge_pose = np.array([0,0,0])
#
#     # Trnasform Function Call
#     vector_tail_hingeframe = transform_vector(vec_in, tail_hinge_pose, tail_hinge_offset)
#
#
#     # Transform to Tail COP
#
#     # Tail dimension ( Span =  32 cm, Root Chord = 22)
#     tail_span = 0.32
#     tail_chord = 0.22
#     # Assuming Centre Of Pressure (COM) for Tail is about *45%* of root chord.
#     tail_cop= 0.1    
#
#     # Transformation Array
#     tail_cop_pose = np.array([ 0.0, elevator_deflection, rudder_deflection])  
#     tail_cop_offset = np.array([tail_cop, 0.0, 0.0]) 
#
#     # Transform Function Call
#     vector_tail_copframe = transform_vector(vector_tail_hingeframe, tail_cop_pose, tail_cop_offset)
#
#     return vector_tail_copframe
