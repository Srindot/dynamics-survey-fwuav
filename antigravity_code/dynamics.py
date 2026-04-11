import numpy as np
from rotation import aero_forces_to_body

class FlappingDynamics:
    def __init__(self, rho=1.225):
        """
        Initialize the structural and aerodynamic solver for the flapping wing.
        :param rho: Air density (kg/m^3)
        """
        self.rho = rho

    def compute_translational_force(self, v_b, v_e, alpha_e, area):
        """
        Computes the translational circulatory force F_tran (Eq 2 and 3).
        """
        # Eq 3: Lift and Drag coefficients
        CL_tran = 1.870 * np.sin(1.872 * alpha_e)
        CD_tran = 1.667 - 1.585 * np.cos(1.968 * alpha_e)

        # Simplified integration over area
        V_squared = np.linalg.norm(v_b + v_e)**2
        
        # Resultant force coefficient C_tran
        C_tran = np.sqrt(CL_tran**2 + CD_tran**2) 
        
        # Eq 2
        F_tran = 0.5 * self.rho * V_squared * C_tran * area
        return F_tran

    def compute_rotational_force(self, omega_lambda, dot_eps_c, r_s, r_c, x_hat_0, area):
        """
        Computes rotational circulation force F_rot (Eq 4 and 6).
        """
        # Eq 6
        C_rot = 2 * np.pi * (0.75 - x_hat_0)
        
        # Eq 4: Integrating over element area
        F_rot = 0.5 * C_rot * self.rho * omega_lambda * dot_eps_c * r_s * r_c * area
        
        return F_rot

    def compute_added_mass_force(self, alpha_e, dot_alpha_e, ddot_alpha_e, omega_lambda, dot_omega_lambda, r_s, r_c, area):
        """
        Computes the added mass force F_add (Eq 7).
        """
        # Eq 7 integrant terms
        term1 = (dot_omega_lambda * np.sin(alpha_e) + omega_lambda * dot_alpha_e * np.cos(alpha_e)) * r_s
        term2 = ddot_alpha_e * (r_c / 4.0)
        
        F_add = (np.pi / 4.0) * self.rho * (r_c**3) * (term1 - term2) * area
        return F_add

    def compute_inertial_force(self, dot_omega_lambda, r_s, rho_w, B_w, area):
        """
        Computes the inertial force F_iner (Eq 8).
        """
        # Eq 8
        F_iner = dot_omega_lambda * r_s * rho_w * B_w * area
        return F_iner
        
    def structural_deformation_strain_matrix(self, nodes, A):
        """
        Compute strain matrix [B] for CST element (Eq 14).
        Nodes: list of 3 tuples (x, y) for the triangle vertices.
        A: Area of triangle
        """
        x1, y1 = nodes[0]
        x2, y2 = nodes[1]
        x3, y3 = nodes[2]
        
        b1 = y2 - y3
        b2 = y3 - y1
        b3 = y1 - y2
        
        c1 = x3 - x2
        c2 = x1 - x3
        c3 = x2 - x1
        
        # Eq 14
        B = (1 / (2 * A)) * np.array([
            [b1, 0, b2, 0, b3, 0],
            [0, c1, 0, c2, 0, c3],
            [c1, b1, c2, b2, c3, b3]
        ])
        
        return B

    def beam_stiffness_matrix(self, E_c, I_c, l_e):
        """
        Compute beam element stiffness matrix (Eq 24).
        """
        coeff = (E_c * I_c) / (l_e**3)
        
        # Eq 24
        K_b = coeff * np.array([
            [12, 6*l_e, -12, 6*l_e],
            [6*l_e, 4*(l_e**2), -6*l_e, 2*(l_e**2)],
            [-12, -6*l_e, 12, -6*l_e],
            [6*l_e, 2*(l_e**2), -6*l_e, 4*(l_e**2)]
        ])
        
        return K_b

    def compute_instantaneous_force(self, F_tran, F_rot, F_add, F_iner):
        """
        Sum of forces (Eq 1).
        """
        return F_tran + F_rot + F_add + F_iner
