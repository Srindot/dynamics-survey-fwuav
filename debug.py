import numpy as np

vel = np.array([6.53, 0.22, 9.21])
rho = 1.225
v_in_sq = vel[0]**2 + vel[2]**2 + 1e-6
v_mag = np.sqrt(v_in_sq)
alpha_mag = np.arctan2(np.abs(vel[2]), np.abs(vel[0]))

C_L = 1.870 * np.sin(1.872 * alpha_mag)
C_D = 1.667 - 1.585 * np.cos(1.968 * alpha_mag) + 1.2

suction_efficiency = np.cos(alpha_mag)**2
C_T_suction = C_L * np.sin(alpha_mag) * suction_efficiency
C_T_friction = C_D * np.cos(alpha_mag)

dS = 0.32 * 0.07
q_dyn = 0.5 * rho * v_in_sq * dS

F_tran_x = -(C_T_friction * q_dyn * np.sign(vel[0])) + (C_T_suction * q_dyn * 1.0)
print(f'Wing Strip F_tran_x: {F_tran_x}')
print(f'alpha_mag: {np.degrees(alpha_mag)}')
print(f'C_T_fric: {C_T_friction}, C_T_suc: {C_T_suction}, q_dyn: {q_dyn}')
