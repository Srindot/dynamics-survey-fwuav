import numpy as np
import matplotlib.pyplot as plt
from kinematics import Kinematics
from dynamics import FlappingDynamics
from rotation import aero_forces_to_body

class FlappingSimulator:
    def __init__(self, dt=0.01):
        """
        Main simulator tying together the 12-DoF kinematics and the Mao 2024 quasi-steady aerodynamics.
        """
        self.dt = dt
        
        # UAV physical params
        self.mass = 0.5 # kg
        self.Jx, self.Jy, self.Jz, self.Jxz = 0.05, 0.05, 0.05, 0.005
        
        self.kin = Kinematics(self.mass, self.Jx, self.Jy, self.Jz, self.Jxz)
        self.dyn = FlappingDynamics(rho=1.225)
        
        # Flapping parameters
        self.amplitude = np.deg2rad(60) 
        self.frequency = 10.0 
        self.alpha_0 = np.deg2rad(15) 

    def run_step(self, t):
        """
        Executes one RK4 step of the simulation.
        """
        state = self.kin.state
        
        lam, omega_lambda, dot_omega_lambda = self.kin.flapping_kinematics(t, self.amplitude, self.frequency)
        theta = lam 
        
        v_b = np.array([self.kin.state[3], self.kin.state[4], self.kin.state[5]]) 
        v_e = np.array([0.0, lam, 0.0]) 
        alpha_e = np.deg2rad(5)
        dot_alpha_e = 0.1
        ddot_alpha_e = 0.01
        dot_eps_c = 0.05
        
        r_s, r_c = 0.1, 0.05 
        x_hat_0 = 0.25 
        area = 0.04 
        rho_w, B_w = 0.1, 0.01 
        
        # Dynamics (Compute Aero Forces via Mao 2024 model)
        F_tran = self.dyn.compute_translational_force(v_b, v_e, alpha_e, area)
        F_rot = self.dyn.compute_rotational_force(omega_lambda, dot_eps_c, r_s, r_c, x_hat_0, area)
        F_add = self.dyn.compute_added_mass_force(alpha_e, dot_alpha_e, ddot_alpha_e, omega_lambda, dot_omega_lambda, r_s, r_c, area)
        F_iner = self.dyn.compute_inertial_force(dot_omega_lambda, r_s, rho_w, B_w, area)
        
        F_inst = self.dyn.compute_instantaneous_force(F_tran, F_rot, F_add, F_iner)
        
        F_T, neg_F_L, F_C = aero_forces_to_body(F_inst, self.alpha_0, theta)
        
        # Multiply by 2 for both wings (assuming symmetric flapping for now)
        F_T *= 2.0
        neg_F_L *= 2.0
        F_C *= 2.0
        
        # Add a simple linear fuselage drag to stabilize the simulation
        drag_coeff = 0.5
        F_drag_x = -drag_coeff * state[3]
        F_drag_y = -drag_coeff * state[4]
        F_drag_z = -drag_coeff * state[5]
        
        # Combine forces [fx, fy, fz, l, m, n]
        forces_moments = np.array([F_T + F_drag_x, F_C + F_drag_y, -neg_F_L + F_drag_z, 0.0, 0.0, 0.0]) 
        
        k1 = self.kin.compute_derivatives(state, forces_moments)
        k2 = self.kin.compute_derivatives(state + 0.5 * self.dt * k1, forces_moments)
        k3 = self.kin.compute_derivatives(state + 0.5 * self.dt * k2, forces_moments)
        k4 = self.kin.compute_derivatives(state + self.dt * k3, forces_moments)
        
        new_state = state + (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        self.kin.state = new_state
        return new_state

if __name__ == "__main__":
    sim = FlappingSimulator(dt=0.01)
    
    # Run the simulation for a longer period to see the dynamics
    time_end = 2.0
    times = np.arange(0, time_end, sim.dt)
    
    # Store history for plotting
    u_hist = []
    w_hist = []
    theta_hist = []
    
    print(f"Running simulation for {time_end}s ({time_end*sim.frequency} flap cycles at {sim.frequency}Hz)...")
    for t in times:
        s = sim.run_step(t)
        u_hist.append(s[3]) # u velocity
        w_hist.append(s[5]) # w velocity
        theta_hist.append(np.rad2deg(s[7])) # theta angle (pitch) in degrees
        
    # Plotting the results
    plt.figure(figsize=(10, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(times, u_hist, 'b-')
    plt.title('Forward Velocity (u) over Time')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(times, w_hist, 'r-')
    plt.title('Vertical Velocity (w) over Time')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(times, theta_hist, 'g-')
    plt.title('Pitch Angle (\u03B8) over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Pitch (deg)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('simulation_results.png')
    print("Graph saved as simulation_results.png")
    
    # Try to show the plot if a display is available
    try:
        plt.show()
    except Exception as e:
        print("Note: Could not display plot interactively in this environment. It has been saved as 'simulation_results.png'.")

