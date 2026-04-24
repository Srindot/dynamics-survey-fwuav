# state_integrator.py : Contains the functions which are used to find the state variables and the pose of the body from the accelerations and the rates.

"""
state_integrator: variables, pose of the body by integrating the accelerations and state_variables in the body frame.
    Functions:
        - integrator

"""

def integrator(current_vector, rate_of_change, dt):
    """
    Generic Euler Integrator for any 6-DOF frame data.
    Works for: 
      - Pose (using Velocity)
      - State (using Acceleration)
    """
    return current_vector + (rate_of_change * dt)
