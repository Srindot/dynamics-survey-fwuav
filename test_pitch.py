import numpy as np
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.kinemaics.state_integrator import integrator
from src.rigid_body_dynamics.dynamics import calculate_state_derivative

# Let's see which way a POSITIVE M_y torque rotates the pitch angle!
force = np.zeros(3)
moment = np.array([0.0, 10.0, 0.0]) # POSITIVE M_y
velocity = np.zeros(3)
rate = np.zeros(3)
pose = np.zeros(6)
mass = 0.5
inertia = np.eye(3)
inertia_inv = np.linalg.inv(inertia)

# 1 second simulation
for _ in range(100):
    accel = calculate_state_derivative(force, moment, velocity, rate, pose, mass, inertia, inertia_inv)
    rate = integrator(rate, accel[3:6], 0.01)
    pose = integrator(pose, np.concatenate([velocity, rate]), 0.01)

print(f"Final Pitch Angle (pose[4]): {np.rad2deg(pose[4]):.2f} degrees")
if pose[4] > 0:
    print("Conclusion: POSITIVE M_y leads to POSITIVE pitch.")
else:
    print("Conclusion: POSITIVE M_y leads to NEGATIVE pitch.")
