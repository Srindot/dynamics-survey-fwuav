import numpy as np

# Test the sign of the elevator and resulting pitch moment
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.kinemaics.frames import TailCOP_Frame, Body_Frame
from src.kinemaics.frame_transform import state_Transform_Tailhinge_to_TailCOP, state_Transform_Body_to_Tailhinge
from src.aerodynamics.aerodynamics import AerodynamicsSolver

tailcop = TailCOP_Frame()
aero = AerodynamicsSolver()

# Drone moving forward at 5m/s (body state vx = 5)
body_state = np.zeros(6)
body_state[0] = 5.0 # moving forward
body_pose = np.zeros(6)

tailhinge_pose = np.array([-0.2, 0, 0, 0, 0, 0])

state_tailhinge = state_Transform_Body_to_Tailhinge(body_state, tailhinge_pose)

# Test Positive Elevator (Trailing Edge Down / Leading Edge Up)
elevator_deflection = np.deg2rad(20) # +20 degrees
tail_vector = np.array([elevator_deflection, 0.0])

state_tailcop = state_Transform_Tailhinge_to_TailCOP(state_tailhinge, tailcop.pull_pose, tail_vector)
tailcop.push_state(state_tailcop)

# Solve forces
aero.solve_tail_forces(tailcop)
wrench = tailcop.pull_wrench
print(f"Elevator = +20 deg -> Tail Fz = {wrench[2]:.2f} N")

if wrench[2] < 0:
    print("Negative Fz (Upward Force). This will push the tail UP, pushing the nose DOWN (Positive Pitch Moment).")
    print("Conclusion: Positive Elevator -> Nose Down Pitch Moment!")
else:
    print("Positive Fz (Downward Force). This will push the tail DOWN, pushing the nose UP (Negative Pitch Moment).")
    print("Conclusion: Positive Elevator -> Nose Up Pitch Moment!")

