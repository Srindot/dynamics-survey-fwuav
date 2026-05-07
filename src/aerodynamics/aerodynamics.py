import numpy as np

class AerodynamicsSolver:
    def __init__(self, rho=1.225, x_0_hat=0.0):
        """
        Blade Element Theory solver using Mao 2024 empirical formulas for Carbon Fiber wings.
        """
        self.rho = rho
        
        # Structural Physics Parameters
        self.x_0_hat = x_0_hat # Pitch axis offset (0 to 1)

    def solve_wing_forces(self, strips, flap_vector):
        """
        Calculates quasi-steady aerodynamic forces on each wing strip.
        Applies Translational Circulation, Rotational Circulation, and updates the local strip wrench.
        """
        flap_rate = flap_vector[1]
        flap_accel = flap_vector[2] if len(flap_vector) > 2 else 0.0

        for strip in strips:
            vel = strip.pull_airspeed  # [v_x, v_y, v_z]
            
            # Local Flow Velocity in XZ plane (Assuming forward flight dominates X, flap dominates Z)
            # Add small epsilon to prevent div by zero
            v_in_sq = vel[0]**2 + vel[2]**2 + 1e-6
            v_mag = np.sqrt(v_in_sq)
            
            # Absolute Geometric Angle of Attack
            alpha_geom = np.arctan2(np.abs(vel[2]), np.abs(vel[0]))
            
            # --- STRUCTURAL TWIST APPROXIMATION ---
            # In NED convention: flap_rate > 0 = wing moves DOWN = DOWNSTROKE
            #                    flap_rate < 0 = wing moves UP   = UPSTROKE
            # During DOWNSTROKE: membrane is pushed taut against veins → no twist
            # During UPSTROKE:   membrane feathers passively → passive twist reduces AOA
            MAX_TWIST = 0.785
            NOMINAL_MAX_FLAP_RATE = 15.0 
            
            if flap_rate < 0:
                # UPSTROKE (wing moving UP): membrane twists passively to shed load
                epsilon_ce = MAX_TWIST * (np.abs(flap_rate) / NOMINAL_MAX_FLAP_RATE)
                epsilon_ce = np.clip(epsilon_ce, 0.0, MAX_TWIST)
                epsilon_ce_dot = (MAX_TWIST / NOMINAL_MAX_FLAP_RATE) * flap_accel
            else:
                # DOWNSTROKE (wing moving DOWN): membrane is taut, full force
                epsilon_ce = 0.0
                epsilon_ce_dot = 0.0

            # True Relative Angle of Attack (alpha_e)
            # The wing twists to align with the flow, reducing the effective AOA.
            alpha_e = np.abs(alpha_geom - epsilon_ce)
            
            # --- 1. Translational Circulation Force (Mao 2024 Eq 2 & 3) ---
            # These empirical equations yield the Lift (C_L) and Drag (C_D) relative to airflow.
            C_L = 1.870 * np.sin(1.872 * alpha_e)
            C_D = 1.667 - 1.585 * np.cos(1.968 * alpha_e)
            
            dS = strip.chord * strip.strip_width
            q_dyn = 0.5 * self.rho * v_in_sq * dS
            
            # Convert to Normal (C_N) and Tangential (C_T) coefficients
            # These are in the STRIP frame (not twisted frame) because alpha_e already
            # accounts for the twist. This is the key insight from Mao 2024.
            C_N = C_D * np.sin(alpha_e) + C_L * np.cos(alpha_e)
            C_T_friction = C_D * np.cos(alpha_e)
            C_T_suction = C_L * np.sin(alpha_e)
            
            # Forces in the STRIP frame (twist already in alpha_e, no extra projection needed)
            # Normal force: perpendicular to chord, sign follows vel[2] direction
            F_N = C_N * q_dyn * np.sign(vel[2])
            
            # Tangential friction: along chord, sign follows vel[0] (opposes motion)
            F_T_friction = C_T_friction * q_dyn * np.sign(vel[0])
            
            # Leading-edge suction: always pulls toward the leading edge (forward = -X in some conventions, +X here)
            # Suction efficiency drops at high AOA
            suction_efficiency = np.cos(alpha_e)**2
            F_T_suction = C_T_suction * suction_efficiency * q_dyn
            
            F_T = F_T_friction + F_T_suction
            
            # --- 2. Rotational Circulation Force (Mao 2024 Eq 4 & 6) ---
            C_rot = 2.0 * np.pi * (0.75 - self.x_0_hat)
            r_ce = strip.chord / 2.0
            r_se = np.abs(strip.y_offset)
            # Rotational force acts normal to chord, opposes the twist rate
            F_rot_z = -C_rot * self.rho * flap_rate * (epsilon_ce_dot * r_se * r_ce) * dS
            
            # --- 3. Added Mass Force (Mao 2024 Eq 7 simplified) ---
            F_add_z = -0.25 * np.pi * self.rho * (strip.chord**2) * (flap_accel * strip.y_offset) * strip.strip_width
            
            # --- 4. Wing Structural Inertial Force (Mao 2024 Eq 8 & Eq 23) ---
            F_iner_z = -strip.strip_mass * (flap_accel * strip.y_offset)
            
            # --- Total Force Assignment ---
            F_tran_x = F_T
            F_tran_z = F_N
            F_tot_x = F_tran_x
            F_tot_y = 0.0
            F_tot_z = F_tran_z + F_rot_z + F_add_z + F_iner_z
            
            # Pitching Moment (Local My) from CoP offset
            # CoP is relative to leading edge
            M_pitch = F_tot_z * strip.chordwise_cop
            
            # SAVE variables for logging
            strip.alpha_geom = alpha_geom
            strip.epsilon_ce = epsilon_ce
            strip.alpha_e = alpha_e
            strip.v_in_sq = v_in_sq
            strip.F_tran_x = F_tran_x
            strip.F_tran_z = F_tran_z
            strip.F_rot_z = F_rot_z
            strip.F_add_z = F_add_z
            strip.F_iner_z = F_iner_z
            strip.F_tot_x = F_tot_x
            strip.F_tot_z = F_tot_z
            strip.M_pitch = M_pitch
            
            # Update the strip's internal wrench
            wrench = np.array([F_tot_x, F_tot_y, F_tot_z, 0.0, M_pitch, 0.0])
            strip.push_wrench(wrench)

    def solve_body_forces(self, body):
        """
        Calculate body parasitic drag and rotational damping.
        Drag: each axis computed independently to prevent cross-coupling.
        Damping: aerodynamic resistance to body rotation (pitch/roll/yaw).
        """
        vel = body.pull_airspeed
        rates = body.pull_state[3:6]  # [p, q, r] angular rates
        
        Cd_body = 1.2
        A_frontal = 0.01  # m^2 frontal area (X direction)
        A_top = 0.05      # m^2 planform area (Z direction)
        
        wrench = np.zeros(6)
        
        # Linear drag per axis
        wrench[0] = 0.5 * self.rho * vel[0] * np.abs(vel[0]) * A_frontal * Cd_body
        wrench[1] = 0.5 * self.rho * vel[1] * np.abs(vel[1]) * A_frontal * Cd_body
        wrench[2] = 0.5 * self.rho * vel[2] * np.abs(vel[2]) * A_top * Cd_body
        
        # Aerodynamic rotational damping (body + tail resist rotation through air)
        # C_damp approximates the torque from the tail and body at ~0.3m from COM
        # M_damp = -C * omega * |omega|  (quadratic damping)
        C_roll_damp = 0.005   # small roll damping
        C_pitch_damp = 0.05   # strong pitch damping (tail is a large surface behind COM)
        C_yaw_damp = 0.02     # moderate yaw damping
        
        wrench[3] = -C_roll_damp * rates[0] * np.abs(rates[0]) * self.rho
        wrench[4] = -C_pitch_damp * rates[1] * np.abs(rates[1]) * self.rho
        wrench[5] = -C_yaw_damp * rates[2] * np.abs(rates[2]) * self.rho
        
        body.push_wrench(wrench)

    def solve_tail_forces(self, tailcop):
        """
        Calculates flat-plate aerodynamics for the tail.
        Uses proper sinusoidal lift model valid at ALL angles (not just small alpha).
        """
        vel = tailcop.pull_airspeed
        
        v_in_sq = vel[0]**2 + vel[2]**2 + 1e-6
        alpha_mag = np.arctan2(np.abs(vel[2]), np.abs(vel[0]))
        
        # Proper flat-plate coefficients valid at all AOA:
        # C_L peaks at ~45° and goes to 0 at 0° and 90° (physically correct)
        C_L = 1.2 * np.sin(2.0 * alpha_mag)
        C_D = 1.28 * np.sin(alpha_mag)**2 + 0.05
        
        # Tail area from shape data
        dS = 0.05  
        q_dyn = 0.5 * self.rho * v_in_sq * dS
        
        # Normal and Tangential coefficients
        C_N = C_D * np.sin(alpha_mag) + C_L * np.cos(alpha_mag)
        C_T_friction = C_D * np.cos(alpha_mag)
        
        F_z = C_N * q_dyn * np.sign(vel[2])
        F_x = C_T_friction * q_dyn * np.sign(vel[0])
        
        wrench = np.array([F_x, 0.0, F_z, 0.0, 0.0, 0.0])
        tailcop.push_wrench(wrench)
