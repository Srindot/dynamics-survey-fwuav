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

    # Transformation Function Call
    vector_bodyframe = transform_vector(vec_in, pose_in, trans_array)

    return vector_bodyframe


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

    # Transform Function Call
    vector_tail_copframe = transform_vector(vec_in, tail_cop_pose, tail_cop_offset)

    return vector_tail_copframe

def vector_Transform_Body_to_Winghinge(vec_in, flapping_angle):
     """
        vector_Transform_Body_to_Winghinge: To Transform a Vector from Body to Winghinge
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - flapping_angle(of): The angle the flapping wing is oriented at the present time around the x-axis.
             Output:
                - vec_out(np.array(3)): Output vector in the output frame.
    """
    # Wing Structural Indidence angle with body
    wing_incidence_angle = np.radians(5)

    # Transformation to Wing Hinge Frame
    wing_hinge_pose = np.array([flapping_angle, wing_incidence_angle, 0.0]) 
    wing_hinge_offset = np.zeros(3)

    # Transformation Function Call
    vec_hingeframe = transform_vector(vec_in, wing_hinge_pose, wing_hinge_offset)

    return vec_hingeframe
    
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

    # Transformation Function Call
    vec_hingeframe = transform_vector(vec_in, wing_strip_pose, wing_strip_offset)

    return vec_hingeframe
    


# Rate Trnasform from Parent to Child 

def rate_Transform_Body_to_Winghinge(rate_in, flapping_angle):
     """
        rate_Transform_Body_to_Winghinge: To Transform the rates from Body to Inertial.
            Input: 
                - rate_in(rx, ry, rz): Input Rate.
                - flapping_angle(ofp): Input angle of flapping wing around x-axis.   
         Output:
                - rate_out(np.array(3)): Output rates in the output frame.
    """
   
    # Wing Hinge Pose
    wing_incidence_angle = np.radians(5)
    pose_arr = np.array([ flapping_angle, wing_incidence_angle, 0.0 ])
    return transform_rate(rate_in, pose_arr)

   
def rate_Transform_Winghinge_to_Wingstrip(rate_in, twist_chordwise, bend_spanwise, flapping_rate):
  """
        vector_Transform_Winghinge_to_Wingstrip: To Transform a Vector from Winghinge to wingstrip.
            Input: 
                - rate_in(rx, ry, rz): Input Rate
                - twist_chordwise(ofc): Each wing strip of BET twisting w.r.t. to y-axis.
                - bend_chordwise(obc): The wing span binding along the wingspan due to incoming and apparent mass forces w.r.t. to x-axis.
           Output:
                - rate_out(np.array(3)): Output rates in the output frame.
    """
    pose_in = np.array([bend_spanwise, twist_chordwise, 0.0])
    rate_out = transform_rate(rate_in, pose_in)
    rate_out[0] += flapping_rate
    return rate_out



# Vector Transform from Child to Parent

def vector_Transform_Body_to_Inertial(vec_in, pose_in):
   """
        vector_Transform_Winghinge_to_Wingstrip: To Transform a Vector from Winghinge to wingstrip.
            Input: 
                - rate_in(rx, ry, rz): Input Rate
                - pose_in(ox, oy, oz): Pose in

             Output:
                - vec_out(np.array(3)): Output vector in the output frame.
    """
 
    # Zero Translation
    trans = np.zeros(3)

    # Function Call and Return the Array
    return transform_vector(vec_in, -pose_in, trans)

def vector_Transform_Tail_to_Body(vec_in, elevator_deflection, rudder_deflection):

    """
        vector_Transform_Tail_to_Body: To Transform a Vector from Tail to Body.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - elevator_deflection: Deflection of the elevator in radians. 
                - rudder_deflection: Deflection of the rudder in radians.
             Output:
                - vec_out(np.array(3)): Output vector in the output frame.
 
    """
 
    # Transformation Array
    tail_cop_pose = np.array([ 0.0, elevator_deflection, rudder_deflection])  
    tail_cop_offset = np.zeros(3) 

    # Transform Function Call
    vector_tail_copframe = transform_vector(vec_in, -tail_cop_pose, tail_cop_offset)

    return vector_tail_copframe

def vector_Transform_Winghinge_to_Body(vec_in, flapping_angle):
     """
        vector_Transform_Tail_to_Body: To Transform a Vector from Tail to Body.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - flapping_angle(ofp): Input angle of flapping wing around x-axis.   
            Output:
                - vec_out(np.array(3)): Output vector in the output frame.
    """   
    # Wing Structural Indidence angle with body
    wing_incidence_angle = np.radians(5)

    # Transformation to Body Frame
    wing_hinge_pose = np.array([flapping_angle, wing_incidence_angle, 0.0]) 
    wing_hinge_offset = np.zeros(3)

    # Transformation Function Call
    vec_hingeframe = transform_vector(vec_in, -wing_hinge_pose, wing_hinge_offset)

    return vec_hingeframe
    
def vector_Transform_Wingstrip_to_Winghinge(vec_in, twist_chordwise, bend_spanwise):
    """
        vector_Transform_Tail_to_Body: To Transform a Vector from Tail to Body.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - twist_chordwise(ofc): Each wing strip of BET twisting w.r.t. to y-axis.
                - bend_chordwise(obc): The wing span binding along the wingspan due to incoming and apparent mass forces w.r.t. to x-axis.
            Output:
                - vec_out(np.array(3)): Output vector in the output frame.
    """   
    # Transformation to Wing hing Frame
    wing_strip_pose = np.array([bend_spanwise, twist_chordwise, 0.0]) 
    wing_strip_offset = np.zeros(3)

    # Transformation Function Call
    vec_hingeframe = transform_vector(vec_in, -wing_strip_pose, wing_strip_offset)

    return vec_hingeframe


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

    # Transformation Function Call
    rate_out = transform_rate(vec_in, -wing_strip_pose)

    return rate_out

def rate_Transform_Body_to_Inertial(rate_in, pose_in):
    """
        rate_Transform_Body_to_Inertial: To Transform the rates from Body to Inertial.
            Input: 
                - vec_in(vx, vy, vz): Input Vector
                - pose_in(ox, oy, oz): Input pose of the coordinate frame of Body w.r.t. to Inertal Frame.  
           Output:
                - rate_out(np.array(3)): Output rates in the output frame.
    """
     
    return transform_rate(rate_in, -pose_in)


  
## ACHRIVE IMPO INFROMATION 
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
#
# d
