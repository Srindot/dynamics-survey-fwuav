# dynamics.py : To define the 6 Dof dynamics for the body in free space acted on by the enternal forces.
import numpy as np

# Rotation Import from the quaternion transform
from ..kinemaics.quaternion_transform import transform_vector


"""

dynamics : To define the 6 Dof dynamics for the body. 
    Functions:
        - calculate_linear_acceleration: Calculates acceleration from the forces.
        - calculate_angular_acceleration: Calculates Angular acceleration from the torques.
        - calculate_state_derivative: Concatenaate and calculates the state aceleration.
        - sum_body_wrenches: Sums all the sub-body frames in body frames.
 
"""



def calculate_linear_acceleration(force_in, velocity_in, rate_in, mass):

    """
    calculate_linear_acceleration: Computes Body-Frame linear acceleration.
    
    Inputs:
        - force_in(np.array): Total summed forces in Body Frame.
        - velocity_in(np.array): Current linear velocity [u, v, w].
        - rate_in(np.array): Current angular rates [p, q, r].
        - mass(float): Vehicle mass.
        
    Output:
        - acc_out(np.array): Linear acceleration vector [du, dv, dw].
    """

    # F = m * a -> a = F / m
    acc_static = force_in / mass

    # Coriolis/Centripetal term: omega x v
    acc_coriolis = np.cross(rate_in, velocity_in)

    # Resultant acceleration in rotating frame
    acc_out = acc_static - acc_coriolis

    return acc_out



def calculate_angular_acceleration(moment_in, rate_in, inertia_matrix, inertia_inv):
    """
    calculate_angular_acceleration: Computes Body-Frame angular acceleration.
    
    Inputs:
        - moment_in(np.array): Total summed moments in Body Frame.
        - rate_in(np.array): Current angular rates [p, q, r].
        - inertia_matrix(np.array): 3x3 Inertia tensor [I].
        - inertia_inv(np.array): 3x3 Inverse inertia tensor [I^-1].
        
    Output:
        - alpha_out(np.array): Angular acceleration vector [dp, dq, dr].
    """

    # Gyroscopic/Centrifugal coupling: omega x (I * omega)
    moment_gyro = np.cross(rate_in, np.dot(inertia_matrix, rate_in))

    # Net moment acting on the inertia
    moment_net = moment_in - moment_gyro

    # Solve for acceleration: alpha = I^-1 * M_net
    alpha_out = np.dot(inertia_inv, moment_net)

    return alpha_out



def calculate_state_derivative(force_in, moment_in, velocity_in, rate_in, pose_in, mass, inertia_matrix, inertia_inv):
    """
    calculate_state_derivative: The  RBD engine that converts physics to accelerations.
    
    Inputs:
        - force_in(np.array): Total aerodynamic/external forces.
        - moment_in(np.array): Total aerodynamic/external moments.
        - velocity_in(np.array): Current [u, v, w].
        - rate_in(np.array): Current [p, q, r].
        - pose_in(np.array): Current 6-DOF body pose.
        - mass(float): Vehicle mass.
        - inertia_matrix, inertia_inv(np.array): Inertia tensors.
        
    Output:
        - accel_out(np.array): The 6-DOF acceleration vector [du, dv, dw, dp, dq, dr].
    """

    # Adding gravity 
    f_gravity = calculate_gravity_force(pose_in, mass)
    f_total = force_in + f_gravity

    # Function Call Linear Acceleration
    v_dot = calculate_linear_acceleration(f_total, velocity_in, rate_in, mass)

    # Function Call for Angular Acceleration
    w_dot = calculate_angular_acceleration(moment_in, rate_in, inertia_matrix, inertia_inv)

    # Concatenaate both angular and linear acceleration
    accel_out = np.concatenate([v_dot, w_dot])

    return accel_out



def calculate_gravity_force(pose_in, mass):
    """
    calculate_gravity_force: Rotates gravity vector into the Body Frame.
    
    Inputs:
        - pose_in(np.array): Current 6-DOF body pose.
        - mass(float): Vehicle mass.
        
    Output:
        - gravity_out(np.array): Gravity force vector in Body Frame.
    """
    # Gravity in Inertial Frame 
    g_inertial = np.array([0.0, 0.0, 9.81 * mass])

    # Extract Rotation from 6-DOF Pose
    rotation = pose_in[3:6]

    # Gravity Transform
    gravity_out = transform_vector(g_inertial, rotation, np.zeros(3))

    return gravity_out



def sum_body_wrenches(wing_hinge_left_wrench, wing_hinge_right_wrench, tail_wrench):
    """
    sum_body_wrenches: Aggregates all external wrenches into a single Body-Frame wrench.
    
    Inputs:
        - wing_hinge_left_wrench (np.array): 6-DOF Wrench from left wing hinge.
        - wing_hinge_right_wrench (np.array): 6-DOF Wrench from right wing hinge.
        - tail_wrench (np.array): 6-DOF Wrench from tail frame.
        
    Output:
        - total_force_out (np.array): Summed forces at Body CoM.
        - total_moment_out (np.array): Summed moments at Body CoM.
    """

    # Direct addition of 6-DOF arrays safely
    total_wrench = wing_hinge_left_wrench + wing_hinge_right_wrench + tail_wrench

    # Split output natively expected by calculate_state_derivative
    total_force_out = total_wrench[:3]
    total_moment_out = total_wrench[3:6]
    
    return total_force_out, total_moment_out
