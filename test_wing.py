import numpy as np
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.kinemaics.frames import Winghinge_Frame, Wing_Strip_Frame, Body_Frame
from src.kinemaics.frame_transform import state_Transform_Body_to_Winghinge, state_Transform_Winghinge_to_Wingstrip
from src.aerodynamics.aerodynamics import AerodynamicsSolver

body = Body_Frame()
body_state = np.zeros(6) # stationary body
hinge = Winghinge_Frame("right")
# Set hinge pose to 0 to be simple
hinge.pose[:3] = [0, 0.2, 0]

strip = Wing_Strip_Frame(chord=0.2, strip_width=0.07, chordwise_cop=0.0, y_offset=0.5, spanwise_bend=0.0)

aero = AerodynamicsSolver()

# Test flap rate = 10 rad/s (Positive -> Upstroke?)
print("=== POSITIVE FLAP RATE ===")
flap_rate = 10.0
flap = np.array([0.0, flap_rate])

hinge_state = state_Transform_Body_to_Winghinge(body_state, hinge.pull_pose, flap)
hinge.push_state(hinge_state)
strip_state = state_Transform_Winghinge_to_Wingstrip(hinge_state, strip.pull_pose)
strip.push_state(strip_state)

vel = strip.pull_state[:3]
print(f"vel: {vel}")
print(f"sign of vel[2]: {np.sign(vel[2])}")

alpha_mag = np.arctan2(np.abs(vel[2]), np.abs(vel[0]))
C_L = 1.872 * np.sin(1.968 * alpha_mag)
C_D = 3.250 * np.sin(1.854 * alpha_mag)**2 + 0.05
C_N = C_D * np.sin(alpha_mag) + C_L * np.cos(alpha_mag)
print(f"C_N: {C_N}")

# Check efficiency
if flap_rate < 0:
    eff = 0.7
else:
    eff = 0.3
print(f"efficiency: {eff}")

C_N *= eff
q_dyn = 0.5 * 1.225 * (vel[0]**2 + vel[2]**2) * (0.2 * 0.07)
F_tran_z = -(C_N * q_dyn * np.sign(vel[2]))

print(f"F_tran_z: {F_tran_z} (Positive = DOWN, Negative = UP)")

print("\n=== NEGATIVE FLAP RATE ===")
flap_rate = -10.0
flap = np.array([0.0, flap_rate])

hinge_state = state_Transform_Body_to_Winghinge(body_state, hinge.pull_pose, flap)
hinge.push_state(hinge_state)
strip_state = state_Transform_Winghinge_to_Wingstrip(hinge_state, strip.pull_pose)
strip.push_state(strip_state)

vel = strip.pull_state[:3]
print(f"vel: {vel}")
print(f"sign of vel[2]: {np.sign(vel[2])}")

alpha_mag = np.arctan2(np.abs(vel[2]), np.abs(vel[0]))
C_L = 1.872 * np.sin(1.968 * alpha_mag)
C_D = 3.250 * np.sin(1.854 * alpha_mag)**2 + 0.05
C_N = C_D * np.sin(alpha_mag) + C_L * np.cos(alpha_mag)
print(f"C_N: {C_N}")

# Check efficiency
if flap_rate < 0:
    eff = 0.7
else:
    eff = 0.3
print(f"efficiency: {eff}")

C_N *= eff
q_dyn = 0.5 * 1.225 * (vel[0]**2 + vel[2]**2) * (0.2 * 0.07)
F_tran_z = -(C_N * q_dyn * np.sign(vel[2]))

print(f"F_tran_z: {F_tran_z} (Positive = DOWN, Negative = UP)")
