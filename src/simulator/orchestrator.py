# orchestrator.py: A super Class which manages and controls the data transfer between the coordinate frame

# Library Imports

import numpy as np

# Transformation Import
from src.kinemaics.frame_transform import (
        state_Transform_Body_to_Winghinge,
        state_Transform_Winghinge_to_Wingstrip,
        state_Transform_Body_to_Tailhinge,
        state_Transform_Tailhinge_to_TailCOP,
        wrench_Transform_Wingstrip_to_Winghinge,
        wrench_Transform_Winghinge_to_Body,
        wrench_Transform_TailCOP_to_Tailhinge,
        wrench_Transform_Tailhinge_to_Body,
        state_Transform_Body_to_Inertial,
        sum_wing_strip_wrenches,
        mirror_wing_wrench
        )

# Coordinate Frame Imports
from src.kinemaics.frames import Inertial_Frame, Body_Frame, Winghinge_Frame, Wing_Strip, Tailhinge_Frame, TailCOP_Frame

# Integrator (Solver)
from src.kinemaics.state_integrator import integrator

# Rigid Body Dynamics
from src.rigid_body_dynamics.dynamics import (
    sum_body_wrenches,
    calculate_state_derivative
)
    
# Actuation Submodule
from src.actuation.actuator import ActuationSystem
    
# Aerodynamics Import 
from src.rigid_body_dynamics.aerodynamics import AerodynamicsSolver


# Super Class   
class Orcehstrator:
    def __init__(self, wing_span=1.4, num_strips=10):

        # Initialize Coordinate Frames

        # Actuator Subsystem
        self.actuator = ActuationSystem(flap_freq=10.0, flap_amplitude=np.deg2rad(60.0))

        # Aerodynamics Subsystem
        # x_0_hat = 0.0 because the 4mm massive leading edge rod acts as the pivot axis
        self.aero_solver = AerodynamicsSolver(rho=1.225, x_0_hat=0.0)

        # Inertial Frame
        self.inertial = Inertial_Frame()

        # Body Frame    
        self.body = Body_Frame()
        
        # Wing Hinge Left & Right
        self.winghinge_l = Winghinge_Frame(side="left")
        self.winghinge_r = Winghinge_Frame(side="right")

        # Wing Properties
        self.strips_l = []
        span_per_wing = wing_span / 2
        strip_width = span_per_wing / num_strips
        
        for i in range(num_strips):

            # Position of the Strip 
            y_ratio = (i + 0.5) / num_strips
            local_y = span_per_wing * y_ratio
            
            # Linear Interpolation: 0.32m at root -> 0.16m at tip
            local_chord = 0.32 - (0.32 - 0.16) * y_ratio
            
            # In flapping wings, the CoP moves dynamically. We permanently anchor the Wing_Strip 
            # coordinate frame to the straight Leading Edge (X=0). The Aerodynamics module will later 
            # calculate the dynamic CoP offset and resolve it into an equivalent aerodynamic Pitching Moment.
            local_cop = 0.0 
            
            strip = Wing_Strip(
                chord=local_chord, 
                strip_width=strip_width, 
                chordwise_cop=local_cop, 
                y_offset=local_y, 
                spanwise_bend=0.0
            )
            
            # --- CUSTOM BAT-WING STRUCTURAL MASS DISTRIBUTION ---
            rho_c = 1600.0
            rho_w = 1420.0  # Cellophane density
            bw = 0.00003    # Cellophane thickness (0.03mm)
            
            # 1. Cellophane Membrane
            m_membrane = rho_w * bw * (strip.chord * strip.strip_width)
            
            # 2. Leading Edge Rod (4mm, spans all strips)
            m_le = rho_c * (np.pi * (0.004**2) / 4.0) * strip.strip_width
            
            # 3. Outer Veins (3mm diagonal + three 1mm radial)
            # Originating from mid-span and spreading outwards to the trailing edge/tips
            m_veins = 0.0
            if i >= num_strips / 2.0:
                # Approximate diagonal length factor (1.5x)
                m_3mm = rho_c * (np.pi * (0.003**2) / 4.0) * (strip.strip_width * 1.5)
                # Three 1mm rods fanning out (approx 1.2x length factor)
                m_1mm_x3 = 3.0 * rho_c * (np.pi * (0.001**2) / 4.0) * (strip.strip_width * 1.2)
                m_veins = m_3mm + m_1mm_x3
                
            strip.strip_mass = m_membrane + m_le + m_veins
            
            self.strips_l.append(strip)
        
        self.tailhinge = Tailhinge_Frame()
        self.tailcop = TailCOP_Frame()

    def down_pass(self, t):
        """ Propagate velocities and states physically from Body outward at time t. """
        # Push simulation time into actuator so it can lazily evaluate
        self.actuator.push_time(t)
        
        body_state = self.body.pull_state
        
        # Actuation Pull: Get instantaneous flapping physics
        flap_vector = self.actuator.pull_flap
        self.winghinge_l.push_flap(flap_vector)
        self.winghinge_r.push_flap(flap_vector)  # Keep right hinge updated for telemetry
        
        # --- LEFT WING ---
        # Body -> Hinge
        state_hinge_l = state_Transform_Body_to_Winghinge(body_state, self.winghinge_l.pull_pose, flap_vector)
        self.winghinge_l.push_state(state_hinge_l)
        
        # Hinge -> Strips
        for strip in self.strips_l:
            state_strip = state_Transform_Winghinge_to_Wingstrip(state_hinge_l, strip.pull_pose)
            strip.push_state(state_strip)
            
        # --- RIGHT WING (Bypassed due to Mirror Logic Optimization) ---
        
        # --- TAIL ---
        # Actuation Pull: Get instantaneous servo deflections
        tail_vector = self.actuator.pull_deflection
        self.tailhinge.push_deflection(tail_vector)
        
        # Body -> Tailhinge
        state_tailhinge = state_Transform_Body_to_Tailhinge(body_state, self.tailhinge.pull_pose)
        self.tailhinge.push_state(state_tailhinge)
        
        # Tailhinge -> TailCOP
        state_tailcop = state_Transform_Tailhinge_to_TailCOP(state_tailhinge, self.tailcop.pull_pose, tail_vector)
        self.tailcop.push_state(state_tailcop)

    def aerodynamics_call(self):
        """ Blade Element Theory (BET) quasi-steady aerodynamic calculations. """
        self.aero_solver.solve_wing_forces(self.strips_l, self.actuator.pull_flap)
        self.aero_solver.solve_tail_forces(self.tailcop)

    def up_pass(self, dt):
        """ Accumulates Wrenches into CoM, computes rigid body solver, steps time via integration. """
        
        # --- LEFT WING ---
        # Delegate loop natively to kinematics helper
        wrench_hinge_l = sum_wing_strip_wrenches(self.strips_l)
        
        self.winghinge_l.push_wrench(wrench_hinge_l)
        flap_vector = self.actuator.pull_flap
        wrench_body_l = wrench_Transform_Winghinge_to_Body(wrench_hinge_l, self.winghinge_l.pull_pose, flap_vector)
        
        # --- RIGHT WING (The Mirror Factor) ---
        # Extract Right Wing Wrench mathematically without recalculating
        wrench_body_r = mirror_wing_wrench(wrench_body_l)
        
        # --- TAIL ---
        tail_vector = self.actuator.pull_deflection
        wrench_tailhinge = wrench_Transform_TailCOP_to_Tailhinge(self.tailcop.pull_wrench, self.tailcop.pull_pose, tail_vector)
        self.tailhinge.push_wrench(wrench_tailhinge)
        
        wrench_body_tail = wrench_Transform_Tailhinge_to_Body(wrench_tailhinge, self.tailhinge.pull_pose)
        
        # --- BODY HUB AND SOLVER ---
        f_tot, m_tot = sum_body_wrenches(wrench_body_l, wrench_body_r, wrench_body_tail)
        self.body.push_wrench(np.concatenate([f_tot, m_tot]))
        
        # Grab Matrices natively from body
        inertia = self.body.pull_inertia
        inertia_inv = np.linalg.inv(inertia)
        
        # Rigid body derivatives 
        accels = calculate_state_derivative(
            f_tot, m_tot,
            self.body.pull_state[:3], self.body.pull_state[3:6],
            self.body.pull_pose,
            self.body.mass, inertia, inertia_inv
        )
        self.body.push_acc(accels)
        
        # Integration Step (Euler explicit calculation)
        new_state = integrator(self.body.pull_state, self.body.pull_acc, dt)
        new_pose = integrator(self.body.pull_pose, new_state, dt)
        
        self.body.push_state(new_state)
        self.body.push_pose(new_pose)
        
        # Inertial Navigation Sync
        inertial_state = state_Transform_Body_to_Inertial(self.body.pull_state, self.body.pull_pose)
        self.inertial.push_state(inertial_state)
